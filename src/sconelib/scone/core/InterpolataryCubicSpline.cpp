/*
** InterpolataryCubicSpline.cpp
*/

#include "InterpolataryCubicSpline.h"
#include "scone/core/Exception.h"
#include <assert.h>

namespace scone
{
	//Here is the old implementation of Natural Cubic Spline in OpenSim, which has been deprecated 
	//as it doesn't correspond to the widely known definition of a Natural Cubic Spline curve. 
	//In Opensim, it has been renamed to "SimmSpline" (OpenSim/Common/SimmSpline.h).
	struct InterpolataryCubicSpline::NatCubSplineImpl {
		/** Array of time values that must be monotonically increasing. */
		std::vector<Real> _x;

		/** Y values. */
		std::vector<Real> _y;

		std::vector<Real> _b;
		std::vector<Real> _c;
		std::vector<Real> _d;

		bool flat_extrapolation;

		void InsertPoint(Real x, Real y) {
			_x.push_back(x);
			_y.push_back(y);
		}

		Real Evalute(Real aX) const {

			// NOT A NUMBER
			if (!_y.size()) return(std::numeric_limits<Real>::quiet_NaN());
			if (!_b.size()) return(std::numeric_limits<Real>::quiet_NaN());
			if (!_c.size()) return(std::numeric_limits<Real>::quiet_NaN());
			if (!_d.size()) return(std::numeric_limits<Real>::quiet_NaN());

			int i, j, k;
			Real dx;

			int n = _x.size();

			/* Check if the abscissa is out of range of the function. If it is,
			* then use the slope of the function at the appropriate end point to
			* extrapolate. You do this rather than printing an error because the
			* assumption is that this will only occur in relatively harmless
			* situations (like a motion file that contains an out-of-range coordinate
			* value). The rest of the SIMM code has many checks to clamp a coordinate
			* value within its range of motion, so if you make it to this function
			* and the coordinate is still out of range, deal with it quietly.
			*/
			if (!flat_extrapolation) {
				if (aX < _x[0])
				{
					return _y[0] + (aX - _x[0])*_b[0];
				}
				else if (aX > _x[n - 1])
				{
					return _y[n - 1] + (aX - _x[n - 1])*_b[n - 1];
				}
			}
			else {
				if (aX < _x[0])
				{
					return _y[0];
				}
				else if (aX > _x[n - 1])
				{
					return _y[n - 1];
				}
			}

			/* Check to see if the abscissa is close to one of the end points
			* (the binary search method doesn't work well if you are at one of the
			* end points.
			*/
			Real TINY_NUMBER = 1e-7;

			if (fabs(aX - _x[0]) < TINY_NUMBER)
			{
				return _y[0];
			}
			else if (fabs(aX - _x[n - 1]) < TINY_NUMBER)
			{
				return _y[n - 1];
			}

			if (n < 3)
			{
				/* If there are only 2 function points, then set k to zero
				* (you've already checked to see if the abscissa is out of
				* range or equal to one of the endpoints).
				*/
				k = 0;
			}
			else
			{
				/* Do a binary search to find which two points the abscissa is between. */
				i = 0;
				j = n;
				while (1)
				{
					k = (i + j) / 2;
					if (aX < _x[k])
						j = k;
					else if (aX > _x[k + 1])
						i = k;
					else
						break;
				}
			}

			dx = aX - _x[k];

			return _y[k] + dx * (_b[k] + dx * (_c[k] + dx * _d[k]));

		}

		void CalcCoefficients()
		{
			Real TINY_NUMBER = 1e-7;
			int n = _x.size();
			int nm1, nm2, i, j;
			Real t;

			if (n < 2)
				return;

			_b.resize(n);
			_c.resize(n);
			_d.resize(n);

			if (n == 2)
			{
				t = std::max(TINY_NUMBER, _x[1] - _x[0]);
				_b[0] = _b[1] = (_y[1] - _y[0]) / t;
				_c[0] = _c[1] = 0.0;
				_d[0] = _d[1] = 0.0;
				return;
			}

			nm1 = n - 1;
			nm2 = n - 2;

			/* Set up tridiagonal system:
			* b = diagonal, d = offdiagonal, c = right-hand side
			*/

			_d[0] = std::max(TINY_NUMBER, _x[1] - _x[0]);
			_c[1] = (_y[1] - _y[0]) / _d[0];
			for (i = 1; i < nm1; i++)
			{
				_d[i] = std::max(TINY_NUMBER, _x[i + 1] - _x[i]);
				_b[i] = 2.0*(_d[i - 1] + _d[i]);
				_c[i + 1] = (_y[i + 1] - _y[i]) / _d[i];
				_c[i] = _c[i + 1] - _c[i];
			}

			/* End conditions. Third derivatives at x[0] and x[n-1]
			* are obtained from divided differences.
			*/

			_b[0] = -_d[0];
			_b[nm1] = -_d[nm2];
			_c[0] = 0.0;
			_c[nm1] = 0.0;

			if (n > 3)
			{
				Real d1, d2, d3, d20, d30, d31;

				d31 = std::max(TINY_NUMBER, _x[3] - _x[1]);
				d20 = std::max(TINY_NUMBER, _x[2] - _x[0]);
				d1 = std::max(TINY_NUMBER, _x[nm1] - _x[n - 3]);
				d2 = std::max(TINY_NUMBER, _x[nm2] - _x[n - 4]);
				d30 = std::max(TINY_NUMBER, _x[3] - _x[0]);
				d3 = std::max(TINY_NUMBER, _x[nm1] - _x[n - 4]);
				_c[0] = _c[2] / d31 - _c[1] / d20;
				_c[nm1] = _c[nm2] / d1 - _c[n - 3] / d2;
				_c[0] = _c[0] * _d[0] * _d[0] / d30;
				_c[nm1] = -_c[nm1] * _d[nm2] * _d[nm2] / d3;
			}

			/* Forward elimination */

			for (i = 1; i < n; i++)
			{
				t = _d[i - 1] / _b[i - 1];
				_b[i] -= t * _d[i - 1];
				_c[i] -= t * _c[i - 1];
			}

			/* Back substitution */

			_c[nm1] /= _b[nm1];
			for (j = 0; j < nm1; j++)
			{
				i = nm2 - j;
				_c[i] = (_c[i] - _d[i] * _c[i + 1]) / _b[i];
			}

			/* compute polynomial coefficients */

			_b[nm1] = (_y[nm1] - _y[nm2]) / _d[nm2] +
				_d[nm2] * (_c[nm2] + 2.0*_c[nm1]);
			for (i = 0; i < nm1; i++)
			{
				_b[i] = (_y[i + 1] - _y[i]) / _d[i] - _d[i] * (_c[i + 1] + 2.0*_c[i]);
				_d[i] = (_c[i + 1] - _c[i]) / _d[i];
				_c[i] *= 3.0;
			}
			_c[nm1] *= 3.0;
			_d[nm1] = _d[nm2];

		}
	};

	//https://en.wikipedia.org/wiki/Cubic_Hermite_spline
	//code converted from 
	//https://github.com/thibauts/cubic-hermite-spline/blob/master/index.js
	struct InterpolataryCubicSpline::CubicHermitSplineImpl {
		/** Array of time values that must be monotonically increasing. */
		std::vector<Real> _x;

		/** Y values. */
		std::vector<Real> _y;

		/** tangent values. */
		std::vector<Real> _tang;

		Real Evalute(Real aX) const {
			const std::vector<Real>& points = _y;
			const std::vector<Real>& tangents = _tang;
			const std::vector<Real>& knots = _x;
			bool derivative = false;

			int n = points.size();    // number or points / tangents / knots
			int i0, i1;
			// find knot interval for t
			int i;
			for (i = 0; i < n - 1; i++) {
				if (aX >= knots[i] && aX <= knots[i + 1]) {
					break;
				}
			}

			SCONE_ERROR_IF(i == n - 1, "out of bounds");

			i0 = i;
			i1 = i + 1;
			Real k0 = knots[i0];
			Real k1 = knots[i1];
			Real scale = k1 - k0;
			Real t = (aX - k0) / scale; //between 0 to 1

			Real h00, h10, h01, h11;
			if (derivative) {
				Real t2 = t * t;
				h00 = 6 * t2 - 6 * t;
				h10 = 3 * t2 - 4 * t + 1;
				h01 = -6 * t2 + 6 * t;
				h11 = 3 * t2 - 2 * t;
			}
			else {
				Real t2 = t * t;
				Real it = 1 - t;
				Real it2 = it * it;
				Real tt = 2 * t;
				h00 = (1 + tt) * it2;
				h10 = t * it2;
				h01 = t2 * (3 - tt);
				h11 = t2 * (t - 1);
			}

			Real 
			v = h00 * points[i0] +
				h10 * tangents[i0] +
				h01 * points[i1] +
				h11 * tangents[i1];

			return v;
		}

	};

	//converted from python code at https://github.com/antdvid/MonotonicCubicInterpolation
	struct InterpolataryCubicSpline::MonotonicCubicInterpolation {
		/** Array of time values that must be monotonically increasing. */
		std::vector<Real> _x;

		/** Y values. */
		std::vector<Real> _y;

		std::vector<Real> _b;
		std::vector<Real> _c;
		std::vector<Real> _d;

		Real Evalute(Real aX) const {
			int n = _x.size();
			SCONE_ERROR_IF(n < 2, "There must be more than 1 coefficient for MonotonicCubicInterpolation");

			// find knot interval for t
			int i;
			for (i = 0; i < n - 1; i++) {
				if (aX >= _x[i] && aX <= _x[i + 1]) {
					break;
				}
			}

			SCONE_ERROR_IF(i == n - 1, "out of bounds");

			i = std::min(i, n - 2);
			Real res = _y[i] + _b[i] * (aX - _x[i]) + _c[i] * pow(aX - _x[i], 2.0) + _d[i] * pow(aX - _x[i], 3.0);

			return res;

		}

		void CalcCoefficients()
		{
			int n = _x.size();
			SCONE_ERROR_IF(n < 2, "There must be more than 1 coefficient for MonotonicCubicInterpolation");

			std::vector<Real> _h, _m;

			_b.resize(n);
			_c.resize(n - 1);
			_d.resize(n - 1);
			_h.resize(n - 1);
			_m.resize(n - 1);

			for (int i = 0; i < n - 1; ++i) {
				_h[i] = _x[i + 1] - _x[i];
				_m[i] = (_y[i + 1] - _y[i]) / _h[i];
			}

			//compute b
			for (int i = 1; i < n; ++i) {
				bool is_mono = (_m[i - 1] * _m[i]) > 0.;
				if (is_mono) {
					_b[i] = 3 * _m[i - 1] * _m[i] / (std::max(_m[i - 1], _m[i]) + 2 * std::min(_m[i - 1], _m[i]));
				}
				else {
					_b[i] = 0;
				}

				if (is_mono && _m[i] > 0) {
					_b[i] = std::min(std::max(0., _b[i]), 3 * std::min(_m[i - 1], _m[i]));
				}
				else if (is_mono && _m[i] < 0) {
					_b[i] = std::max(std::min(0., _b[i]), 3 * std::max(_m[i - 1], _m[i]));
				}
			}

			_b[0] = ((2 * _h[0] + _h[1]) * _m[0] - _h[0] * _m[1]) / (_h[0] + _h[1]);
			_b[n - 1] = ((2 * _h[n - 2] + _h[n - 3]) * _m[n - 2] - _h[n - 2] * _m[n - 3]) / (_h[n - 2] + _h[n - 3]);


			for (int i = 0; i < n - 1; ++i) {
				_c[i] = (3 * _m[i] - _b[i + 1] - 2 * _b[i]) / _h[i];
				_d[i] = (_b[i + 1] + _b[i] - 2 * _m[i]) / (_h[i] * _h[i]);
			}
		}

	};

	InterpolataryCubicSpline::InterpolataryCubicSpline(const PropNode& props, Params& par) :
		total_time(0.0)
	{
		INIT_PROP(props, spline_type, String("NatCubSpline"));
		INIT_PROP(props, control_points, size_t(0));
		INIT_PROP(props, offset_time, 0.0);
		INIT_PROP(props, scale_time, 1.0);
		INIT_PROP(props, flat_extrapolation, false);
		INIT_PROP(props, zero_start, false);
		INIT_PROP(props, zero_end, false);
		INIT_PROP(props, cyclic, true);

		control_point_y = props.try_get_any_child({ "control_point_y" });//default value for all coefficients
		control_point_dt = props.try_get_any_child({ "control_point_dt" });//default value for all dt
		control_point_tangent = props.try_get_any_child({ "control_point_tangent" });//default value for all tangents

		if (zero_start || zero_end)  flat_extrapolation = true;

		auto sct = props.try_get_child(stringf("scale_time"));
		if (sct ) {
			scale_time = par.get(stringf("ScaleTime"), *sct);
		}

		auto offt = props.try_get_child(stringf("offset_time"));
		if (offt) {
			offset_time = par.get(stringf("OffsetTime"), *offt);
		}

		std::vector<Real> times;
		std::vector<Real> values;
		std::vector<Real> tangents;

		int csid = 0;
		if (zero_start) csid++; //add an zero point at the beginning

		int ceid = 0;
		if (zero_end) 	ceid++; //add an zero point at the end

		for (int cpidx = 0; cpidx < control_points + csid + ceid; //total # of control points 
			++cpidx)
		{
			Real xVal = 0.0;//first time  = 0, shall we consider non-zero?
			if (cpidx > 0) //increment the time
			{
				double dt = 0.0;
				auto cc = props.try_get_child(stringf("delta_time%d", cpidx - 1));
				if (cc) //check if the specified dt exists
					dt = par.get(stringf("DT%d", cpidx - 1), *cc);			
				else if(control_point_dt) //otherwise use default
					dt = par.get(stringf("DT%d", cpidx - 1), *control_point_dt); //time interval
				else
					SCONE_ERROR("Could not find control_point_dt object");;

				xVal = times.back() + dt;
			}

			Real yVal = 0.0;
			if (cpidx == 0 && zero_start) {//first one with zero start
				yVal = 0.0;
			}
			else if ((cpidx == control_points + csid + ceid -1) && zero_end) {//last one with zero end
				yVal = 0.0;
			}
			else {//cpidx is not the 0 or the last
				auto cc = props.try_get_child(stringf("coefficient%d", cpidx-csid));
				if (cc) {
					yVal = par.get(stringf("Y%d", cpidx - csid), *cc);
				}
				else if (control_point_y)
					yVal = par.get(stringf("Y%d", cpidx - csid), *control_point_y);
				else
					SCONE_ERROR("Could not find control_point_y object");;
			}

			times.push_back(xVal);
			values.push_back(yVal);

			//for cubic hermite spline, tangents are needed
			if (spline_type == "CubicHermiteSpline") {
				Real tVal = 0.0;
				auto cc = props.try_get_child(stringf("tangent%d", cpidx));
				if (cc) {
					tVal = par.get(stringf("T%d", cpidx), *cc);
				}
				else if (control_point_tangent)
					tVal = par.get(stringf("T%d", cpidx), *control_point_tangent);
				else
					SCONE_ERROR("Could not find control_point_tangent object");;

				tangents.push_back(tVal);
			}
		}

		total_time = (times.back() - times.front()) * scale_time;

		if (spline_type == "NatCubSpline") {
			m_pImpl = std::make_unique<NatCubSplineImpl>();
			m_pImpl->flat_extrapolation = flat_extrapolation;
			m_pImpl->_x = times;
			m_pImpl->_y = values;
			m_pImpl->CalcCoefficients();
		}
		else if (spline_type == "CatmullRomSpline") {
			INIT_PROP(props, tangent_begin_type, String("Linear"));
			INIT_PROP(props, tangent_end_type, String("Linear"));

			std::vector<Real> cvalues(values.size() + 2); //need the extra beginning and end points
			std::copy(values.begin(), values.end(), cvalues.begin() + 1);
			
			if (tangent_begin_type == "Zero") {
				tangent_begin_value = values[1];
			}
			else if (tangent_begin_type == "Linear") {
				tangent_begin_value = values.front();
			}
			else {
				auto cc = props.try_get_child(stringf("tangent_begin_value"));
				if (cc) //check if the specified parameter exists
					tangent_begin_value = par.get(stringf("tangent_begin_value"), *cc);
				else {
					tangent_begin_value = values.front();
				}
			}

			if (tangent_end_type == "Zero") {
				tangent_end_value = values[values.size()-2];
			}
			else if (tangent_end_type == "Linear") {
				tangent_end_value = values.back();
			}
			else {
				auto cc = props.try_get_child(stringf("tangent_end_value"));
				if (cc) //check if the specified parameter exists
					tangent_end_value = par.get(stringf("tangent_end_value"), *cc);
				else {
					tangent_end_value = values.back();
				}
			}

			cvalues.front() = tangent_begin_value;
			cvalues.back()  = tangent_end_value;

			Real alpha = 0.5; //tension
			m_cmrImpl = std::make_unique< xo::catmull_rom<Real, Real>>(cvalues, times, alpha);
		}
		else if (spline_type == "CubicHermiteSpline") {
			m_chsImpl = std::make_unique<CubicHermitSplineImpl>();
			m_chsImpl->_x = times;
			m_chsImpl->_y = values;
			m_chsImpl->_tang = tangents;
		}
		else if (spline_type == "MonotonicCubicInterpolation") {
			m_mciImpl = std::make_unique<MonotonicCubicInterpolation>();
			m_mciImpl->_x = times;
			m_mciImpl->_y = values;
			m_mciImpl->CalcCoefficients();
		}
		else {
			SCONE_ERROR("Could not find spline_type " + spline_type);
		}

	}

	InterpolataryCubicSpline::~InterpolataryCubicSpline()
	{
	}

	scone::Real InterpolataryCubicSpline::GetValue(Real x)
	{
		Real spl_total = total_time / scale_time; //total time in spline time domain
		Real pt = (x + offset_time) / scale_time; //offset_time could be negative, pt is time in the spline domain
		if (cyclic) {
			if (pt > spl_total || pt < 0.0) {
				pt = fmod(pt, spl_total); //only for cylic
			}
			if (pt < 0.0) pt += spl_total;
		}
		else {
			if (pt > spl_total) {
				return 0.0;
			}
		}
		//if (pt < 0.0) pt += spl_total;

		Real v = 0.0;
		if(m_pImpl) v = m_pImpl->Evalute(pt);
		else if(m_cmrImpl) v = (*m_cmrImpl)(pt);
		else if (m_chsImpl) v = m_chsImpl->Evalute(pt);
		else if (m_mciImpl) v = m_mciImpl->Evalute(pt);
		else {
			SCONE_ERROR("Could not use spline_type " + spline_type);		
			return 0.0;
		}

		return v;
	}

	String InterpolataryCubicSpline::GetSignature()
	{
		return stringf("InterpolataryCubicSpline%d", control_points);
	}
	
}
