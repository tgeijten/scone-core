#include "SpinalController.h"

#include "scone/model/Model.h"
#include "snel/update.h"
#include "scone/model/Sensors.h"
#include "scone/model/MuscleId.h"
#include "scone/model/Muscle.h"
#include "scone/core/Log.h"
#include "scone/core/profiler_config.h"
#include "snel/snel_tools.h"

namespace scone
{
	using xo::uint32;

	SpinalController::SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		INIT_MEMBER_REQUIRED( pn, neural_delays_ ),
		INIT_MEMBER_REQUIRED( pn, activation_ )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		InitMuscleInfo( pn, model );

		// create delayed sensors and actuators
		for ( auto& mus : model.GetMuscles() ) {
			auto nd = GetNeuralDelay( *mus );
			auto& sp = model.AcquireSensor<MuscleLengthSensor>( *mus );
			delayed_spindle_sensors_.push_back( model.GetDelayedSensor( sp, nd ) );
			delayed_actuators_.push_back( model.GetDelayedActuator( *mus, nd ) );
		}

		// add neuron groups
		spindle_group_ = AddMuscleNeurons( "L", pn, par );
		auto ia_group = AddGroupNeurons( "IA", pn, par );
		motor_group_ = AddMuscleNeurons( "MN", pn, par );

		// connect ia interneurons
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			auto& mg = muscle_groups_[ mgi ];
			for ( auto mi : mg.muscle_indices_ )
				Connect( spindle_group_, mi, ia_group, mgi, par, pn, "L_IA_weight", mg.muscle_indices_.size() );
			for ( auto amgi : mg.ant_group_indices_ )
				Connect( ia_group, amgi, ia_group, mgi, par, pn, "IA_IA_weight", mg.ant_group_indices_.size() );
		}

		// connect motor units
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			Connect( spindle_group_, mi, motor_group_, mi, par, pn, "L_MN_weight", 1 );
			for ( auto amgi : muscles_[ mi ].ant_group_indices_ )
				Connect( ia_group, amgi, motor_group_, mi, par, pn, "IA_MN_weight", muscles_[ mi ].ant_group_indices_.size() );
		}
	}

	bool SpinalController::ComputeControls( Model& model, double timestamp )
	{
		auto& muscles = model.GetMuscles();

		for ( uint32 mi = 0; mi < muscles.size(); ++mi )
			network_.set_value( spindle_group_, mi, snel::real( delayed_spindle_sensors_[ mi ].GetValue() ) );

		network_.update();

		for ( uint32 mi = 0; mi < muscles.size(); ++mi )
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
			muscles_.emplace_back( mus->GetName() );

		for ( auto& [key, mgpn] : pn.select( "MuscleGroup" ) ) {
			for ( auto side : { Side::Left, Side::Right } ) {
				auto& mg = muscle_groups_.emplace_back( mgpn, side );
				auto& muscle_pattern = mgpn.get<xo::pattern_matcher>( "muscles" );
				for ( uint32 mi = 0; mi < muscles_.size(); ++mi )
					if ( mg.side_ == muscles_[ mi ].side_ && muscle_pattern.match( muscles_[ mi ].name_ ) )
						mg.muscle_indices_.emplace_back( mi );
				if ( mg.muscle_indices_.empty() )
					muscle_groups_.pop_back();
			}
		}

		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			for ( auto mi : muscle_groups_[ mgi ].muscle_indices_ )
				muscles_[ mi ].group_indices_.push_back( mgi );
			auto antagonists = muscle_groups_[ mgi ].pn_.try_get<xo::pattern_matcher>( "antagonists" );
			for ( uint32 amgi = 0; amgi < muscle_groups_.size(); ++amgi ) {
				if ( muscle_groups_[ mgi ].side_ == muscle_groups_[ amgi ].side_ ) {
					if ( antagonists && antagonists->match( muscle_groups_[ amgi ].name_ ) ) {
						muscle_groups_[ mgi ].ant_group_indices_.emplace_back( amgi );
						for ( auto mi : muscle_groups_[ mgi ].muscle_indices_ )
							muscles_[ mi ].ant_group_indices_.push_back( amgi );
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

	snel::neuron_id SpinalController::AddNeuron( snel::group_id group, String name, Params& par, const PropNode& pn, const string& pinf )
	{
		auto bias = !pinf.empty() ? par.try_get( GetNameNoSide( name ), pn, pinf, 0.0 ) : 0.0;
		return AddNeuron( group, name, bias );
	}

	snel::group_id SpinalController::AddMuscleNeurons( String name, const PropNode& pn, Params& par )
	{
		auto gid = network_.add_group( snel::get_update_fn( pn.get<string>( name + "_activation", activation_ ) ) );
		auto bias_inf = name + "_bias";
		for ( auto& mus : muscles_ )
			AddNeuron( gid, name + '.' + mus.name_, par, pn, bias_inf );
		return gid;
	}

	snel::group_id SpinalController::AddGroupNeurons( String name, const PropNode& pn, Params& par )
	{
		auto gid = network_.add_group( snel::get_update_fn( pn.get<string>( name + "_activation", activation_ ) ) );
		auto bias_inf = name + "_bias";
		for ( auto& mg : muscle_groups_ )
			AddNeuron( gid, name + '.' + mg.sided_name(), par, pn, bias_inf );
		return gid;
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, uint32 sidx, snel::group_id tgid, uint32 tidx, Real weight )
	{
		return network_.connect( sgid, sidx, tgid, tidx, snel::real( weight ) );
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, uint32 sidx, snel::group_id tgid, uint32 tidx, Params& par, const PropNode& pn, const string& pinf, size_t size )
	{
		SCONE_ASSERT( size > 0 );
		auto snid = network_.get_id( sgid, sidx );
		auto tnid = network_.get_id( tgid, tidx );
		auto par_name = GetNameNoSide( neuron_names_[ tnid.value() ] ) + "-" + GetNameNoSide( neuron_names_[ snid.value() ] );
		auto weight = par.try_get( par_name, pn, pinf, 0.0 ) / Real( size );
		return Connect( sgid, sidx, tgid, tidx, weight );
	}
}
