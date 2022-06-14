/*
** ComPivotReflex.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Reflex.h"

namespace scone
{
	/// EXPERIMENTAL reflex that is subject to change; please don't use.
	/// Must be part of ReflexController.
	class ComPivotReflex : public Reflex
	{
	public:
		ComPivotReflex( const PropNode& props, Params& par, Model& model, const Location& loc );
		virtual ~ComPivotReflex();

		virtual void ComputeControls( double timestamp ) override;

		/// Name of the pivot Body, append with _o for Body on opposite side.
		String pivot_body;
		/// Direction; default = [ 0 0 1 ]
		Vec3 dir;
		/// Mirror the dof value for left sided reflexes, useful for pelvis_list, pelvis_rotation, etc.; default = 0.
		bool mirror_left;

		/// Target position; default = 0.
		Real P0;
		/// Position feedback gain; default = 0.
		Real KP;
		/// Allow this reflex to be negative; default = 1.
		bool allow_neg_P;

		/// Target velocity; default = 0.
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
		bool m_Mirror;
		Body* m_SourceBody;
		SensorDelayAdapter& m_DelayedPos;
		SensorDelayAdapter& m_DelayedVel;
	};
}
