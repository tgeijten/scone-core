#include "SpinalController.h"

#include "scone/model/Model.h"
#include "snel/update.h"
#include "scone/model/Sensors.h"
#include "scone/model/MuscleId.h"
#include "scone/model/Muscle.h"
#include "scone/core/Log.h"

namespace scone
{
	using xo::uint32;

	SpinalController::SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		INIT_MEMBER_REQUIRED( pn, neural_delays_ )
	{
		InitMuscleInfo( pn, model );

		// create delayed sensors and actuators
		for ( auto& mus : model.GetMuscles() ) {
			auto nd = GetNeuralDelay( *mus );
			auto& sp = model.AcquireSensor<MuscleLengthSensor>( *mus );
			delayed_spindle_sensors_.push_back( model.GetDelayedSensor( sp, nd ) );
			delayed_actuators_.push_back( model.GetDelayedActuator( *mus, nd ) );
		}

		// add neuron groups
		spindle_group_ = network_.add_group( snel::update_linear );
		for ( auto& minf : muscle_infos_ )
			AddNeuron( spindle_group_, "L_" + minf.name_, 0.0 );

		auto ia_group = network_.add_group( snel::update_tanh_norm );
		for ( auto& mg : muscle_groups_ )
			AddNeuron( ia_group, "IA_" + mg.sided_name(), par.try_get( "IA_" + mg.name_ + ".C0", pn, "ia_bias", 0.0 ) );

		motor_group_ = network_.add_group( snel::update_tanh_norm );
		for ( auto& minf : muscle_infos_ )
			AddNeuron( motor_group_, "MN_" + minf.name_, par.try_get( GetNameNoSide( minf.name_ ) + ".C0", pn, "mn_bias", -0.3 ) );

		// connect ia interneurons
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			auto& mg = muscle_groups_[ mgi ];
			double muscle_scale = 1.0 / mg.muscle_indices_.size();
			for ( auto mi : mg.muscle_indices_ )
				Connect( spindle_group_, mi, ia_group, mgi,
					muscle_scale * par.try_get( "IA_" + mg.name_ + "." + GetNameNoSide( muscle_infos_[ mi ].name_ ), pn, "l_ia_weight", 0.5 ) );
			double group_scale = 1.0 / mg.antaganist_group_indices_.size();
			for ( auto amgi : mg.antaganist_group_indices_ )
				Connect( ia_group, amgi, ia_group, mgi,
					group_scale * par.try_get( "IA_" + mg.name_ + "." + "IA_" + muscle_groups_[ amgi ].name_, pn, "ia_ia_weight", -0.5 ) );
		}

		// connect motor units
		for ( uint32 mi = 0; mi < muscle_infos_.size(); ++mi ) {
			Connect( spindle_group_, mi, motor_group_, mi, par.try_get( GetNameNoSide( muscle_infos_[ mi ].name_ ) + ".L", pn, "ia_mono", 0.5 ) );
			auto scale = 1.0 / muscle_infos_[ mi ].antaganist_group_indices_.size();
			for ( auto amgi : muscle_infos_[ mi ].antaganist_group_indices_ )
				Connect( ia_group, amgi, motor_group_, mi,
					scale * par.try_get( GetNameNoSide( muscle_infos_[ mi ].name_ ) + ".IA_" + muscle_groups_[ amgi ].name_, pn, "ia_mn_weight", -0.5 ) );
		}
	}

	bool SpinalController::ComputeControls( Model& model, double timestamp )
	{
		auto& muscles = model.GetMuscles();

		for ( xo::uint32 mi = 0; mi < muscles.size(); ++mi )
			network_.set_value( spindle_group_, mi, snel::real( delayed_spindle_sensors_[ mi ].GetValue() ) );

		network_.update();

		for ( xo::uint32 mi = 0; mi < muscles.size(); ++mi )
			delayed_actuators_[ mi ].AddInput( network_.value( motor_group_, mi ) );

		return false;
	}

	void SpinalController::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const {
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		for ( index_t i = 0; i < network_.neuron_count(); ++i )
			frame[ "sn." + neuron_names_[ i ] ] = network_.values_[ i ];
	}

	PropNode SpinalController::GetInfo() const { return PropNode(); }
	String SpinalController::GetClassSignature() const { return "SN"; }

	void SpinalController::InitMuscleInfo( const PropNode& pn, Model& model )
	{
		// setup muscle info
		for ( auto& mus : model.GetMuscles() )
			muscle_infos_.emplace_back( mus->GetName() );

		for ( auto& [key, mgpn] : pn.select( "MuscleGroup" ) ) {
			for ( auto side : { Side::Left, Side::Right } ) {
				auto& mg = muscle_groups_.emplace_back( mgpn, side );
				auto& muscle_pattern = mgpn.get<xo::pattern_matcher>( "muscles" );
				for ( uint32 mi = 0; mi < muscle_infos_.size(); ++mi )
					if ( mg.side_ == muscle_infos_[ mi ].side_ && muscle_pattern.match( muscle_infos_[ mi ].name_ ) )
						mg.muscle_indices_.emplace_back( mi );
			}
		}

		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			for ( auto mi : muscle_groups_[ mgi ].muscle_indices_ )
				muscle_infos_[ mi ].group_indices_.push_back( mgi );
			auto antagonists = muscle_groups_[ mgi ].pn_.try_get<xo::pattern_matcher>( "antagonists" );
			for ( uint32 amgi = 0; amgi < muscle_groups_.size(); ++amgi ) {
				if ( muscle_groups_[ mgi ].side_ == muscle_groups_[ amgi ].side_ ) {
					if ( antagonists && antagonists->match( muscle_groups_[ amgi ].name_ ) ) {
						muscle_groups_[ mgi ].antaganist_group_indices_.emplace_back( amgi );
						for ( auto mi : muscle_groups_[ mgi ].muscle_indices_ )
							muscle_infos_[ mi ].antaganist_group_indices_.push_back( amgi );
					}
				}
			}
		}
	}

	TimeInSeconds SpinalController::GetNeuralDelay( const Muscle& m ) const
	{
		auto it = neural_delays_.find( MuscleId( m.GetName() ).base_ );
		SCONE_ERROR_IF( it == neural_delays_.end(), "Could not find neural delay for " + m.GetName() );
		return it->second;
	}

	snel::neuron_id SpinalController::AddNeuron( snel::group_id group, String name, Real bias )
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		neuron_names_.emplace_back( std::move( name ) );
		return network_.add_neuron( group, snel::real( bias ) );
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Real weight )
	{
		return network_.connect( sgid, sidx, tgid, tidx, snel::real( weight ) );
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Real scale, Params& par, const PropNode& pn, const string& pinf )
	{
		auto snid = network_.get_id( sgid, sidx );
		auto tnid = network_.get_id( tgid, tidx );
		auto par_name = GetNameNoSide( neuron_names_[ tnid.value() ] ) + "." + GetNameNoSide( neuron_names_[ snid.value() ] );
		auto weight = scale * par.try_get( par_name, pn, pinf, 0.0 );
		return network_.connect( sgid, sidx, tgid, tidx, snel::real( weight ) );
	}
}
