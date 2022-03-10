/*
** DofReflex.h
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Reflex.h"

namespace scone
{
	/// Reflex based on the value of a specific DOF.
	/// Must be part of ReflexController.
	class BodyOrientationReflex : public Reflex
	{
	public:
		BodyOrientationReflex( const PropNode& props, Params& par, Model& model, const Location& loc );
		virtual ~BodyOrientationReflex();

		virtual void ComputeControls( double timestamp ) override;

		/// Name of the DOF that is the source of this Reflex, append with _o for DOF on opposite side.
		String source;
		/// Orientation axis; default = [ 0 0 1 ]
		Vec3 axis;
		/// Name of the axis; default = ""
		String axis_name;

		/// Target orientation [rad]; default = 0.
		Real P0;
		/// Orientation feedback gain; default = 0.
		Real KP;
		/// Allow this reflex to be negative; default = 1.
		bool allow_neg_P;

		/// Target angular velocity [rad or m]; default = 0.
		Real V0;
		/// Velocity feedback gain; default = 0.
		Real KV;
		/// Allow this reflex to be negative; default = 1.
		bool allow_neg_V;

		/// Constant actuation added to the reflex; default = 0.
		Real C0;

		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;

	private:
		Real u_p;
		Real u_v;
		Body& m_SourceBody;
		SensorDelayAdapter& m_DelayedPos;
		SensorDelayAdapter& m_DelayedVel;
	};
}
