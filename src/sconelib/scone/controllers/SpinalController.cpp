#include "SpinalController.h"

#include "scone/model/Model.h"
#include "snel/update.h"
#include "scone/model/Sensors.h"
#include "scone/model/MuscleId.h"
#include "scone/model/Muscle.h"

namespace scone
{
	MuscleGroup::MuscleGroup( const PropNode& pn ) :
		INIT_MEMBER_REQUIRED( pn, name_ ),
		INIT_MEMBER( pn, antagonists_, {} ),
		INIT_MEMBER( pn, synergists_, {} ),
		INIT_MEMBER( pn, linked_, {} )
	{}

	NeuronGroup::NeuronGroup( const PropNode& pn ) :
		INIT_MEMBER_REQUIRED( pn, type_ )
	{}

	NeuronLink::NeuronLink( const PropNode& pn, SpinalController& sc ) :
		INIT_MEMBER_REQUIRED( pn, type_ ),
		INIT_MEMBER_REQUIRED( pn, activation_ )
	{}

	SpinalController::SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		INIT_MEMBER_REQUIRED( pn, neural_delays_ ),
		muscle_groups_( pn.get_all<MuscleGroup>( "MuscleGroup" ) )
	{
		auto& muscles = model.GetMuscles();

		// create delayed sensors and actuators
		for ( auto& mus : muscles ) {
			auto nd = GetNeuralDelay( *mus );
			auto sp = model.AcquireSensor<MuscleLengthSensor>( *mus );
			delayed_spindle_sensors_.push_back( model.GetDelayedSensor( sp, nd ) );
			delayed_actuators_.push_back( model.GetDelayedActuator( *mus, nd ) );
		}

		// add neuron groups
		spindle_group_ = nn_.add_group( muscles.size(), snel::update_linear );
		auto ia_group = nn_.add_group( muscle_groups_.size() * 2, snel::update_relu );
		motor_group_ = nn_.add_group( muscles.size(), snel::update_relu );

		// connect neurons
		for ( xo::uint32 mi = 0; mi < muscles.size(); ++mi ) {
			nn_.connect( spindle_group_, mi, motor_group_, mi );
		}

		for ( index_t mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
		}
	}

	bool SpinalController::ComputeControls( Model& model, double timestamp )
	{
		auto& muscles = model.GetMuscles();

		for ( xo::uint32 mi = 0; mi < muscles.size(); ++mi )
			nn_.set_value( spindle_group_, mi, snel::real( delayed_spindle_sensors_[ mi ].GetValue() ) );

		nn_.update();

		for ( xo::uint32 mi = 0; mi < muscles.size(); ++mi )
			delayed_actuators_[ mi ].AddInput( nn_.value( motor_group_, mi ) );

		return false;
	}

	void SpinalController::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const {}
	PropNode SpinalController::GetInfo() const { return PropNode(); }

	String SpinalController::GetClassSignature() const
	{
		return "SN";
	}

	TimeInSeconds SpinalController::GetNeuralDelay( const Muscle& m ) const
	{
		auto it = neural_delays_.find( MuscleId( m.GetName() ).base_ );
		SCONE_ERROR_IF( it == neural_delays_.end(), "Could not find neural delay for " + m.GetName() );
		return it->second;
	}
}
