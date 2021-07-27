/*
** InterpolataryCubicSpline.h
** Author(s): Alex Zhou
*/

#pragma once

#include "Function.h"
#include "scone/core/string_tools.h"
#include "PropNode.h"
#include "scone/optimization/Params.h"
#include <xo/geometry/catmull_rom.h>

namespace scone
{
	/// Parameterizable interpolatory cubic spline function. 
	class SCONE_API InterpolataryCubicSpline : public Function
	{
	public:
		InterpolataryCubicSpline( const PropNode& props, Params& par );
		virtual ~InterpolataryCubicSpline();

		enum SplineType {NatCubSpline = 0, CatmullRomSpline = 1, CubHermite = 2, MonotonicCub = 3};
		String spline_type;

		/// for Catmull Rom spline, additional points before the first point and after the last point 
		/// are needed to determine the tangents at those locations
		String tangent_begin_type;
		String tangent_end_type;
		Real   tangent_begin_value;
		Real   tangent_end_value;

		/// Number of control points in this function.
		size_t control_points;

		/// Parameter for the y value of each control point.
		const PropNode* control_point_y;

		/// Parameter for the dt (delta time [s] from previous point) value of each control point.
		const PropNode* control_point_dt;

		/// Parameter for the tangent or slope value of each control point.
		const PropNode* control_point_tangent;

		/// Parameter for the offset time.
		const PropNode* offset_time_node;
		
		/// Parameter for the scale time.
		const PropNode* scale_time_node;

		/// Flag indicating if value should stay flat after passing the last control point; default = false.
		/// It will be automatically changed to true if either zero_start or zero_end is set to true
		bool flat_extrapolation;

		/// Flag indicating if the initial value shall be zero
		bool zero_start;

		/// Flag indicating if the end value shall be zero
		bool zero_end;

		/// Offset timing
		Real offset_time;

		/// scale timing
		Real scale_time;

		/// Flag indicating if the spline repeating itself continuously 
		bool cyclic;

		virtual Real GetValue(Real x) override;
		virtual String GetSignature() override;

	protected:
		struct NatCubSplineImpl;
		u_ptr< NatCubSplineImpl > m_pImpl;

		struct CubicHermitSplineImpl;
		u_ptr< CubicHermitSplineImpl > m_chsImpl;

		u_ptr<xo::catmull_rom<Real, Real>> m_cmrImpl; //the tangent of a point is parallel to the line connecting the point before and after it

		struct MonotonicCubicInterpolation;
		u_ptr< MonotonicCubicInterpolation> m_mciImpl;

		Real total_time;
	};
}
