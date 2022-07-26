#include "ExampleController.h"

#include "scone/model/Model.h"
#include "scone/model/Actuator.h"
#include "scone/core/Log.h"

namespace scone
{
	ExampleController::ExampleController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		INIT_PAR_MEMBER( pn, par, actuator_offset, 0.0 )
	{
	}

	bool ExampleController::ComputeControls( Model& model, double timestamp )
	{
		for ( auto& act : model.GetActuators() )
			act->AddInput( actuator_offset );

		return false;
	}

	String ExampleController::GetClassSignature() const
	{
		return "Example";
	}
}
