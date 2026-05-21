/*
** PieceWiseLinearFunction.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Function.h"
#include "PropNode.h"
#include "scone/optimization/Params.h"
#include "xo/numerical/piecewise_linear_function.h"

namespace scone
{
	/// Parameterizable piece-wise linear function.
	class SCONE_API PieceWiseLinearFunction : public Function
	{
	public:
		PieceWiseLinearFunction( const PropNode& props, Params& par );
		virtual ~PieceWiseLinearFunction() = default;

		/// Number of control points in this function.
		size_t control_points;

		/// Parameter for the y value of each control point.
		const PropNode& control_point_y;

		/// Parameter for the dt (delta time [s] from previous point) value of each control point.
		const PropNode* control_point_dt;

		/// Parameter for the time of the first control point; default = 0.
		TimeInSeconds control_point_t0;

		/// Flag indicating if value should stay flat after passing the last control point; default = false.
		bool flat_extrapolation;

		virtual Real GetValue( Real x ) override;
		virtual String GetSignature() override;

	private:
		xo::piecewise_linear_function<Real> m_Func;
	};
}
