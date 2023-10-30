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
	/// Implementation of the two joined sine curve for raise and fall profile function. 
	/// For more details, see: Ding, Walsh et al., Human-in-the-loop optimization of hip assistance with a soft exosuit during walking,
	/// Science Robotics, 2018.
	class SCONE_API RaiseFallProfileFunction : public Function
	{
	public:
		RaiseFallProfileFunction( const PropNode& props, Params& par );
		virtual ~RaiseFallProfileFunction();

		/// Profile start time [s]; default = not set
		const PropNode& start_time;

		/// Profile raise time [s]; default = not set
		const PropNode& raise_time;

		/// Profile fall time [s]; default = not set
		const PropNode& fall_time;

		/// Profile peak; default = not set
		const PropNode& peak;

		virtual Real GetValue( Real x ) override;
		virtual String GetSignature() override;

	protected:
		struct SineImpl;
		u_ptr< SineImpl > m_pSineImpl;
	};
}
