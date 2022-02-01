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

	uint32 sided_idx( uint32 idx, Side s ) { return idx * 2 + uint32( s == Side::Left ); }

	MuscleGroup::MuscleGroup( const PropNode& pn ) :
		INIT_MEMBER_REQUIRED( pn, name_ ),
		INIT_MEMBER_REQUIRED( pn, muscles_ ),
		INIT_MEMBER( pn, antagonists_, {} ),
		INIT_MEMBER( pn, synergists_, {} )
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
			log::debug( "adding ", mus->GetName(), " nd=", nd );
			delayed_spindle_sensors_.push_back( model.GetDelayedSensor( sp, nd ) );
			//delayed_actuators_.push_back( model.GetDelayedActuator( *mus, nd ) );
		}

		// add neuron groups
		spindle_group_ = network_.add_group( muscles.size(), snel::update_linear );

		auto ia_group = network_.add_group( muscle_groups_.size() * 2, snel::update_relu );
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi )
			network_.set_bias( ia_group, mgi, float( par.try_get( muscle_groups_[ mgi ].name_ + ".C0", pn, "ia_bias", 0.0 ) ) );

		motor_group_ = network_.add_group( muscles.size(), snel::update_relu );
		for ( uint32 mi = 0; mi < muscles.size(); ++mi )
			network_.set_bias( motor_group_, mi, float( par.try_get( muscles[ mi ]->GetName() + ".C0", pn, "mn_bias", 0.0 ) ) );

		// monosynaptic connections
		for ( uint32 mi = 0; mi < muscles.size(); ++mi ) {
			auto lid = network_.connect( spindle_group_, mi, motor_group_, mi );
			network_.set_weight( lid, float( par.try_get( muscles[ mi ]->GetName() + ".L", pn, "ia_mono", 0.0 ) ) );
		}

		// connect ia interneurons
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			for ( uint32 mi = 0; mi < muscles.size(); ++mi ) {
				auto& mg = muscle_groups_[ mgi ];
				auto& mus = muscles[ mi ];
				for ( auto side : { Side::Right, Side::Left } )	{
					if ( side == mus->GetSide() && mg.muscles_.match( mus->GetName() ) ) {
						auto lid = network_.connect( spindle_group_, mi, ia_group, sided_idx( mi, side ) );
						auto par_name = "Ia." + mg.name_ + "." + muscles[ mi ]->GetName();
						network_.set_weight( lid, float( par.try_get( par_name, pn, "l_ia_weight", 0.0 ) ) );
					}

					//if ( side == mus->GetSide() && mg.antagonists_.match( mus->GetName() ) ) {
					//	auto lid = network_.connect( ia_group, sided_idx( mi, side ), motor_group_, mi );
					//	auto par_name = muscles[ mi ]->GetName() + ".Ia." + mg.name_;
					//	network_.set_weight( lid, float( par.try_get( muscles[ mi ]->GetName() + ".L", pn, "ia_mn_weight", 0.0 ) ) );
					//}
				}
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
