/*
** RaiseFallProfileFunction.h
** Author(s): Alex Zhou
*/

#pragma once

#include "Function.h"
#include "scone/core/string_tools.h"
#include "PropNode.h"
#include "scone/optimization/Params.h"

namespace scone
{
	/// Parameterized raise-fall function. 
	class SCONE_API RaiseFallProfileFunction : public Function
	{
	public:
		RaiseFallProfileFunction( const PropNode& props, Params& par );
		virtual ~RaiseFallProfileFunction();

		const PropNode& start_time;
		const PropNode& raise_time;
		const PropNode& fall_time;
		const PropNode& peak;

		virtual Real GetValue( Real x ) override;
		virtual String GetSignature() override;

	protected:
		struct SineImpl;
		u_ptr< SineImpl > m_pSineImpl;
	};
}
