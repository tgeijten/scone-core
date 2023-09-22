/*
** DofMeasure.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Measure.h"
#include "RangePenalty.h"
#include "scone/core/Angle.h"
#include "scone/model/Dof.h"

namespace scone
{
	/// Measure that adds a penalty for a specific degree-of-freedom (DoF).
	/** Penalties can be based on ''position'', ''velocity'', ''acceleration'', ''limit_torque'' and ''actuator_torque''. Example:
	\verbatim
	# Measure for upper body
	DofMeasure {
		dof = pelvis_tilt
		position { min = -45 max = 0 abs_penalty = 10 } # Penalize leaning backwards
		velocity { min = -10 max = 10 abs_penalty = 10 } # Penalize upper body motion
	}
	# Measure for knee limits
	CompositeMeasure {
		name = DofLimits
		symmetric = true
		DofMeasure {
			dof = knee_angle
			threshold = 2
			threshold_transition = 2
			weight = 0.1
			limit_torque { min = 0 max = 0 abs_penalty = 1 }
		}
	}
	\endverbatim
	*/
	class DofMeasure : public Measure
	{
	public:
		DofMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc );
		virtual double ComputeResult( const Model& model ) override;
		virtual double GetCurrentResult( const Model& model ) override;
		virtual void Reset( Model& model ) override;

		/// Dof to which to apply the penalty to.
		Dof& dof;

		/// Optional parent dof which will be added to the dof value.
		Dof* parent;

		/// Set if rotations are in degrees or radians; default = 1
		bool in_degrees;

		/// Penalty for when the DOF position [deg or m] is out of range.
		RangePenalty<Real> position;

		/// Penalty for when the DOF velocity [deg/s or m/s] is out of range.
		RangePenalty<Real> velocity;

		/// Penalty for when the DOF acceleration [deg/s%%^%%2 or m/s%%^%%2] is out of range.
		RangePenalty<Real> acceleration;

		/// Penalty for when the DOF limit torque [Nm] is out of range (this value is signed!).
		RangePenalty<Real> limit_torque;

		/// Penalty for the DOF actuator torque (this value is signed!).
		RangePenalty<Real> actuator_torque;

	protected:
		virtual bool UpdateMeasure( const Model& model, double timestamp ) override;
		virtual String GetClassSignature() const override;
		virtual void StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const override;

	private:
		int range_count;
		Real ConvertDofValue( Real value ) const { return in_degrees ? Radian( value ).deg_value() : value; }
	};
}
