/*
** Reflex.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/core/types.h"
#include "scone/core/math.h"
#include "scone/core/HasData.h"
#include "scone/core/PropNode.h"
#include "scone/model/Location.h"
#include "scone/optimization/Params.h"
#include "scone/core/ValuePtrMap.h"

namespace scone
{
	/// Base class for reflexes, requires use of ReflexController. See inherited Controllers for details.
	class Reflex : public HasData
	{
	public:
		Reflex( const PropNode& props, Params& par, Model& model, ReflexController& rc, const Location& loc );
		virtual ~Reflex();

		/// Name of the target actuator; use _o for actuators on the opposite side.
		String target;

		/// Minimum output for this reflex; default = -infinity.
		Real min_control_value;

		/// Maximum output for this reflex; default = +infinity.
		Real max_control_value;

		/// Neuromuscular delay [s] used for this reflex; default = 0.
		TimeInSeconds delay;

		virtual void ComputeControls( double timestamp );
		virtual void StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const override {}

		int TrySetControlParameter( const String& name, Real value );
		xo::optional<Real> TryGetControlParameter( const String& name );
		std::vector<String> GetControlParameters() const;

	protected:
		/// clamp control value between min_control_value and max_control_value and add to target actuator
		Real AddTargetControlValue( Real u );
		Actuator& actuator_;
		static String GetReflexName( const String& target, const String& source );
		static String GetParName( const PropNode& props, const Location& loc );

		RealPtrMap controls_;
	};
}
