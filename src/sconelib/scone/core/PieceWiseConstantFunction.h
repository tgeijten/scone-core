/*
** PieceWiseConstantFunction.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Function.h"
#include "PropNode.h"
#include "scone/optimization/Params.h"
#include "xo/numerical/piecewise_constant_function.h"

namespace scone
{
	/// Parameterizable piece-wise constant function.
	class SCONE_API PieceWiseConstantFunction : public Function
	{
	public:
		PieceWiseConstantFunction( const PropNode& props, Params& par );
		virtual ~PieceWiseConstantFunction() = default;

		/// Number of control points in this function.
		size_t control_points;

		/// Parameter for the y value of each control point.
		const PropNode& control_point_y;

		/// Parameter for the dt value of each control point.
		const PropNode* control_point_dt;

		/// Parameter for the time of the first control point; default = 0.
		TimeInSeconds control_point_t0;

		virtual Real GetValue( Real x ) override;

		// a signature describing the function
		virtual String GetSignature() override;

	private:
		xo::piecewise_constant_function<Real> m_Func;
	};
}
