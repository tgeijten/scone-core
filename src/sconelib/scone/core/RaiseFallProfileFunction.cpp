/*
** RaiseFallProfileFunction.cpp
*/

#include "RaiseFallProfileFunction.h"
#include "scone/core/Exception.h"
#include "scone/core/math.h"
#include <cmath>

namespace scone
{
	//This implements the two joined sine curve for raise and fall profile function
	//see Ding, Walsh et al. Human-in-the-loop optimization of hip assistance with a soft exosuit during walking
	//Science Robotics, 2018
	struct RaiseFallProfileFunction::SineImpl {
		Real start_time;
		Real raise_time;
		Real fall_time;
		Real peak;

		Real Evalute(Real aX) const {

			const Real mPI = REAL_PI;
			if (aX < start_time) return 0.0;
			if (aX < start_time + raise_time) {
				Real x = (aX - start_time)/raise_time*0.5*mPI;
				return peak * sin(x);
			}
			if (aX < start_time + raise_time+fall_time) {
				Real x = (aX - start_time-raise_time) / fall_time * 0.5*mPI;
				return peak * cos(x);
			}
			return  0.0;
		}
	};

	RaiseFallProfileFunction::RaiseFallProfileFunction(const PropNode& props, Params& par) :
		start_time(props.get_child("start_time")),
		raise_time(props.get_child("raise_time")),
		fall_time(props.get_child("fall_time")),
		peak(props.get_child("peak")),
		m_pSineImpl(new SineImpl)
	{
		m_pSineImpl->start_time = par.get("StartTime", start_time);
		m_pSineImpl->raise_time = par.get("RaiseTime", raise_time);
		m_pSineImpl->fall_time = par.get("FallTime", fall_time);
		m_pSineImpl->peak = par.get("Peak", peak);
	}

	RaiseFallProfileFunction::~RaiseFallProfileFunction()
	{
	}

	scone::Real RaiseFallProfileFunction::GetValue(Real x)
	{
		return m_pSineImpl->Evalute(x);
	}

	String RaiseFallProfileFunction::GetSignature()
	{
		return "RaiseFallProfileFunction";
	}
	
}
