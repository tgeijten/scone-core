/*
** SensorReflex.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Reflex.h"

namespace scone
{
	/// Generic Reflex based on a single sensor input.
	/// Must be part of ReflexController.
	class SensorReflex : public Reflex
	{
	public:
		SensorReflex( const PropNode& props, Params& par, Model& model, const Location& loc, SensorDelayAdapter& s );

		/// Mirror the dof value for left sided reflexes, useful for pelvis_list, pelvis_rotation, etc.; default = 0.
		bool mirror_left;

		/// Offset value, subtracted from input; default = 0.
		Real P0;
		/// Position feedback gain; default = 0.
		Real KP;
		/// Allow this reflex to be negative; default = 1.
		bool allow_neg_P;

		/// Constant actuation added to the reflex; default = 0.
		Real C0;

		virtual void ComputeControls( double timestamp ) override;
		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;

	private:
		Real val_;
		Real sign_;
		SensorDelayAdapter& source_;
	};
}
