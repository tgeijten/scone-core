#include "SensorNeuron.h"
#include "spot/par_tools.h"
#include "scone/model/Locality.h"
#include "scone/model/Model.h"
#include "scone/model/Sensors.h"
#include "scone/model/SensorDelayAdapter.h"
#include "scone/core/string_tools.h"
#include "scone/model/Muscle.h"
#include "scone/model/Dof.h"
#include "NeuralController.h"
#include "xo/utility/hash.h"

namespace scone
{
	SensorNeuron::SensorNeuron( const PropNode& pn, Params& par, NeuralController& nc, const String& name, Index idx, Side side, const String& act_func ) :
	Neuron( pn, idx, side, act_func ),
	input_(),
	sensor_gain_( 1.0 ),
	type_( pn.get< string >( "type" ) ),
	sample_delay_frames_( 0 ),
	sample_delay_window_( 21 ),
	use_sample_delay_( false )
	{
		bool inverted = pn.get< bool >( "inverted", false );
		par_name_ = GetNameNoSide( name ) + ( inverted ? "-." : "." ) + type_;

		ScopedParamSetPrefixer sp( par, par_name_ );
		delay_ = pn.get< double >( "delay", nc.GetDelay( GetNameNoSide( name ) ) );
		sample_delay_frames_ = std::lround( delay_ / nc.GetModel().GetSimulationStepSize() );
		offset_ = par.try_get( "0", pn, "offset", type_ == "L" ? 1 : ( inverted ? 1 : 0 ) );
		sensor_gain_ = inverted ? -1 : 1;
		Dof* dof = nullptr;

		auto& model = nc.GetModel();

		switch( xo::hash( type_ ) )
		{
		case "F"_hash:
			muscle_ = FindByName( model.GetMuscles(), name ).get();
			input_ = &nc.GetModel().AcquireDelayedSensor< MuscleForceSensor >( *muscle_ );
			break;
		case "L"_hash:
			muscle_ = FindByName( model.GetMuscles(), name ).get();
			input_ = &nc.GetModel().AcquireDelayedSensor< MuscleLengthSensor >( *muscle_ );
			break;
		case "S"_hash:
			muscle_ = FindByName( model.GetMuscles(), name ).get();
			input_ = &nc.GetModel().AcquireDelayedSensor< MuscleSpindleSensor >( *muscle_ );
			use_sample_delay_ = true;
			break;
		case "U"_hash:
			muscle_ = FindByName( model.GetMuscles(), name ).get();
			input_ = &nc.GetModel().AcquireDelayedSensor< MuscleExcitationSensor >( *muscle_ );
			break;
		case "DP"_hash:
			dof = FindByName( model.GetDofs(), name ).get();
			input_ = &nc.GetModel().AcquireDelayedSensor< DofPositionSensor >( *dof );
			break;
		case "DV"_hash:
			dof = FindByName( model.GetDofs(), name ).get();
			input_ = &nc.GetModel().AcquireDelayedSensor< DofVelocitySensor >( *dof );
			break;
		case "DPV"_hash:
			dof = FindByName( model.GetDofs(), name ).get();
			auto kv = par.get( ".DV", 0.1, 0.01, 0, 1 );
			auto parent_name = pn.try_get_any< string >( { "parent", "source_parent" } );
			Dof* root_dof = parent_name ? FindByName( model.GetDofs(), *parent_name ).get() : nullptr;
			input_ = &nc.GetModel().AcquireDelayedSensor< DofPosVelSensor >( *dof, kv, root_dof );
			break;
		}

		// mirror sensor gain for right side sensors of MirrorDofs
		if ( dof && nc.IsMirrorDof( *dof ) && side == RightSide )
			sensor_gain_ *= -1;

		xo_error_if( !input_, "Unknown type " + type_ );
		source_name_ = name;
	}

	double SensorNeuron::GetOutput( double offset ) const
	{
		auto input = use_sample_delay_ ? input_->GetAverageValue( sample_delay_frames_, sample_delay_window_ ) : input_->GetValue( delay_ );
		return output_ = activation_function( sensor_gain_ * ( input - offset_ - offset ) );
	}

	string SensorNeuron::GetName( bool mirrored ) const
	{
		if ( !mirrored )
			return source_name_ + "." + type_;
		else return GetMirroredName( source_name_ ) + "." + type_;
	}

	string SensorNeuron::GetParName() const
	{
		return par_name_;
	}
}
