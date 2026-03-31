/*
** JointMeasure.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Measure.h"
#include "RangePenalty.h"
#include "scone/core/Angle.h"
#include "scone/model/Joint.h"

namespace scone
{
	/// Measure for penalizing specific muscle quantities.
	/** Penalties can be based on muscle input, activation, length and velocity. Example:
	\verbatim
	JointMeasure {
		joint = knee
		limit_torque { max = 0 squared_penalty = 1 } # penalize squared limit torque
	}
	\endverbatim
	*/
	class JointMeasure : public Measure
	{
	public:
		JointMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc );
		virtual double ComputeResult( const Model& model ) override;
		virtual double GetCurrentResult( const Model& model ) override;
		virtual void Reset( Model& model ) override;

		/// Joint to which to apply the penalty to.
		Joint& joint;

		/// Penalty for the joint force [N].
		RangePenalty<Real> joint_force;

		/// Penalty for when the Joint limit torque [Nm] is out of range.
		RangePenalty<Real> limit_torque;

		/// Penalty for the Joint actuator torque [Nm].
		RangePenalty<Real> muscle_torque;

		/// Penalty for the Joint actuator torque [Nm].
		RangePenalty<Real> motor_torque;

	protected:
		virtual UpdateResult UpdateMeasure( const Model& model, double timestamp ) override;
		virtual String GetClassSignature() const override;
		virtual void StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const override;

		std::array<std::pair<RangePenalty<Real>*, const char*>, 4> penalties;
	};
}