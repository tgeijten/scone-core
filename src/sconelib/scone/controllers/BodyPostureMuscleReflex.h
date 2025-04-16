/*
** BodyPostureMuscleReflex.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Reflex.h"
#include "scone/core/Angle.h"
#include "scone/model/Sensors.h"

namespace scone
{
	/// Reflex that rotates the source Body towards a target orientation, based on the muscle moment arm
	///
	/// Must be part of ReflexController. The excitation is computed as follows:
	/// ''U = C0 + KP rot(source, muscle, joint) + KV ang_vel(source, muscle, joint)'', 
	/// where ''rot(source, muscle, joint)'' and ''ang_vel(source, muscle, joint)'' are the rotation of the ''source'' body from its ''target_orientation''
	/// and its angular velocity, in the direction of the moment arm of the ''target'' muscle with respect to the ''joint''.
	class BodyPostureMuscleReflex : public Reflex
	{
	public:
		BodyPostureMuscleReflex( const PropNode& props, Params& par, Model& model, ReflexController& rc, const Location& loc );
		virtual ~BodyPostureMuscleReflex();

		virtual void ComputeControls( double timestamp ) override;

		/// Name of the Body that is the source of this Reflex, append with _o for Body on opposite side.
		String source;
		/// Optional name of the Joint that is used to compute the muscle moment arm; default = autodetect
		String joint;
		/// Target orientation in YZX Euler order; default = [ 0 0 0 ]
		Vec3Deg target_orientation;

		/// Orientation feedback gain; default = 0.
		Real KP;
		/// Velocity feedback gain; default = 0.
		Real KV;

		/// Constant actuation added to the reflex; default = 0.
		Real C0;

		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;

	private:
		Real u_;
		Body& m_SourceBody;
		SensorDelayAdapter* m_BodyPostureMuscleSensor;
	};
}
