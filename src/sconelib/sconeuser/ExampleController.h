#pragma once

#include "scone/controllers/Controller.h"

namespace scone
{
	class ExampleController : public Controller
	{
	public:
		ExampleController( const PropNode& pn, Params& par, Model& model, const Location& loc );
		virtual ~ExampleController() = default;

		Real actuator_offset;

	protected:
		virtual bool ComputeControls( Model& model, double timestamp ) override;
		virtual String GetClassSignature() const override;
	};
}
