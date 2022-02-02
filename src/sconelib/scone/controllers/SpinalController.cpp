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

	MuscleGroup::MuscleGroup( const PropNode& pn, const Model& model, Side side, uint32 idx ) :
		pn_( pn ), name_( pn.get_str( "name" ) ), side_( side ), index_( idx )
	{
		auto& muscles = model.GetMuscles();
		auto& muscle_pattern = pn_.get<xo::pattern_matcher>( "muscles" );
		for ( uint32 mi = 0; mi < muscles.size(); ++mi )
			if ( side_ == muscles[ mi ]->GetSide() && muscle_pattern.match( muscles[ mi ]->GetName() ) )
				muscle_indices_.emplace_back( mi );
	}

	void MuscleGroup::Initialize( const Model& model, const std::vector<MuscleGroup>& muscle_groups )
	{
		auto& muscles = model.GetMuscles();
		auto antagonists = pn_.try_get<xo::pattern_matcher>( "antagonists" );
		for ( uint32 amgi = 0; amgi < muscle_groups.size(); ++amgi ) {
			if ( side_ == muscle_groups[ amgi ].side_ ) {
				if ( antagonists && antagonists->match( muscle_groups[ amgi ].name_ ) ) {
					antaganist_group_indices_.emplace_back( amgi );
					xo::append( antaganist_muscle_indices_, muscle_groups[ amgi ].muscle_indices_ );
				}
			}
		}
		string str = "antagonists of " + name_ + ":";
		for ( auto ami : antaganist_muscle_indices_ )
			str += " " + muscles[ ami ]->GetName();
		log::debug( str );
	}

	NeuronGroup::NeuronGroup( const PropNode& pn ) :
		INIT_MEMBER_REQUIRED( pn, type_ )
	{}

	NeuronLink::NeuronLink( const PropNode& pn, SpinalController& sc ) :
		INIT_MEMBER_REQUIRED( pn, type_ ),
		INIT_MEMBER_REQUIRED( pn, activation_ )
	{}

	SpinalController::SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		INIT_MEMBER_REQUIRED( pn, neural_delays_ )
	{
		// setup muscle groups
		for ( auto& [key, mgpn] : pn.select( "MuscleGroup" ) )
			for ( auto side : { Side::Left, Side::Right } )
				muscle_groups_.emplace_back( mgpn, model, side, uint32( muscle_groups_.size() ) );
		for ( auto& mg : muscle_groups_ )
			mg.Initialize( model, muscle_groups_ );

		// create delayed sensors and actuators
		auto& muscles = model.GetMuscles();
		for ( auto& mus : muscles ) {
			auto nd = GetNeuralDelay( *mus );
			auto& sp = model.AcquireSensor<MuscleLengthSensor>( *mus );
			delayed_spindle_sensors_.push_back( model.GetDelayedSensor( sp, nd ) );
			delayed_actuators_.push_back( model.GetDelayedActuator( *mus, nd ) );
		}

		// add neuron groups
		spindle_group_ = network_.add_group( snel::update_linear );
		for ( auto& mus : muscles )
			AddNeuron( spindle_group_, "L_" + mus->GetName(), 0.0 );

		auto ia_group = network_.add_group( snel::update_tanh_norm );
		for ( auto& mg : muscle_groups_ )
			AddNeuron( ia_group, "IA_" + mg.sided_name(), par.try_get( "IA_" + mg.name_ + ".C0", pn, "ia_bias", 0.0 ) );

		motor_group_ = network_.add_group( snel::update_tanh_norm );
		for ( auto& mus : muscles )
			AddNeuron( motor_group_, "MN_" + mus->GetName(), par.try_get( GetNameNoSide( mus->GetName() ) + ".C0", pn, "mn_bias", -0.3 ) );

		// connect ia interneurons
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			auto& mg = muscle_groups_[ mgi ];
			double muscle_scale = 1.0 / mg.muscle_indices_.size();
			for ( auto mi : mg.muscle_indices_ )
				Connect( spindle_group_, mi, ia_group, mgi,
					muscle_scale * par.try_get( "IA_" + mg.name_ + "." + GetNameNoSide( muscles[ mi ]->GetName() ), pn, "l_ia_weight", 0.5 ) );
			double group_scale = 1.0 / mg.antaganist_group_indices_.size();
			for ( auto amgi : mg.antaganist_group_indices_ )
				Connect( ia_group, amgi, ia_group, mgi,
					group_scale * par.try_get( "IA_" + mg.name_ + "." + "IA_" + muscle_groups_[ amgi ].name_, pn, "ia_ia_weight", -0.5 ) );
		}

		// connect motor units
		for ( uint32 mi = 0; mi < muscles.size(); ++mi ) {
			Connect( spindle_group_, mi, motor_group_, mi, par.try_get( GetNameNoSide( muscles[ mi ]->GetName() ) + ".L", pn, "ia_mono", 0.5 ) );
			for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
				if ( xo::contains( muscle_groups_[ mgi ].antaganist_muscle_indices_, mi ) )
					Connect( ia_group, mgi, motor_group_, mi,
						par.try_get( GetNameNoSide( muscles[ mi ]->GetName() ) + ".IA_" + muscle_groups_[ mgi ].name_, pn, "ia_mn_weight", -0.5 ) );
			}
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

	String SpinalController::GetClassSignature() const
	{
		return "SN";
	}

	snel::neuron_id SpinalController::AddNeuron( snel::group_id group, String name, Real bias )
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		neuron_names_.emplace_back( std::move( name ) );
		return network_.add_neuron( group, snel::real( bias ) );
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Real weight )
	{
		auto snid = network_.get_id( sgid, sidx );
		auto tnid = network_.get_id( tgid, tidx );
		log::debug( neuron_names_[ snid.value() ], " -> ", neuron_names_[ tnid.value() ], " weight=", weight );
		return network_.connect( sgid, sidx, tgid, tidx, snel::real( weight ) );
	}

	TimeInSeconds SpinalController::GetNeuralDelay( const Muscle& m ) const
	{
		auto it = neural_delays_.find( MuscleId( m.GetName() ).base_ );
		SCONE_ERROR_IF( it == neural_delays_.end(), "Could not find neural delay for " + m.GetName() );
		return it->second;
	}
}
