#include "ExternalController.h"
#include "scone/model/Model.h"
#include "scone/model/Actuator.h"

namespace scone
{
	ExternalController::ExternalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		actuator_count_( model.GetActuators().size() )
	{}

	void ExternalController::SetInput( index_t idx, Real input )
	{
		if ( inputs_.empty() )
			inputs_.resize( actuator_count_ );
		SCONE_ASSERT( idx < inputs_.size() );
		inputs_[ idx ] = input;
	}

	void ExternalController::SetDelayedInput( index_t idx, Real input )
	{
		if ( delayed_inputs_.empty() )
			delayed_inputs_.resize( actuator_count_ );
		SCONE_ASSERT( idx < delayed_inputs_.size() );
		delayed_inputs_[ idx ] = input;
	}

	void ExternalController::Reset( Model& model )
	{
		inputs_.clear();
		delayed_inputs_.clear();
	}

	bool ExternalController::ComputeControls( Model& model, double timestamp )
	{
		if ( !inputs_.empty() )
		{
			auto& act = model.GetActuators();
			SCONE_ASSERT( inputs_.size() == act.size() );
			for ( index_t idx = 0; idx < inputs_.size(); ++idx )
				act[ idx ]->AddInput( inputs_[ idx ] );
		}
		if ( !delayed_inputs_.empty() )
		{
			if ( delayed_actuators_.empty() )
				for ( auto& a : model.GetActuators() )
					delayed_actuators_.emplace_back(
						model.GetDelayedActuator( *a, model.GetTwoWayNeuralDelay( *a ) ) );

			SCONE_ASSERT( delayed_inputs_.size() == delayed_actuators_.size() );
			for ( index_t idx = 0; idx < delayed_inputs_.size(); ++idx )
				delayed_actuators_[ idx ].AddInput( delayed_inputs_[ idx ] );
		}
		return false;
	}

	String ExternalController::GetClassSignature() const
	{
		return String();
	}
}
