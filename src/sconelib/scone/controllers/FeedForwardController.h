/*
** FeedForwardController.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/core/types.h"
#include "scone/controllers/Controller.h"
#include "scone/core/PropNode.h"
#include "scone/optimization/Params.h"
#include "scone/core/Function.h"
#include "scone/model/Leg.h"

namespace OpenSim
{
	class PiecewiseLinearFunction;
}

namespace scone
{
	/// Controller that produces a feed-forward control signal for any actuator, based on a Function.
	class FeedForwardController : public Controller
	{
	public:
		FeedForwardController( const PropNode& props, Params& par, Model& model, const Location& target_area );
		virtual ~FeedForwardController() { };

		/// Bool indicating if function should be the same for left and right; default = true.
		bool symmetric;

		/// Actuator names to include (semicolon separated); default = "*"
		String include;

		/// Actuator names to exclude (semicolon separated); default = ""
		String exclude;

		virtual bool ComputeControls( Model& model, double timestamp ) override;
		virtual String GetClassSignature() const override;

	private:
		struct ActuatorInfo {
			ActuatorInfo( Actuator* act ) : actuator( act ), function_idx( no_index ) {}
			Actuator* actuator;
			index_t function_idx;
		};

		std::vector<FunctionUP> functions_;
		std::vector<ActuatorInfo> act_infos_;
		std::vector<Real> function_results_;
	};
}
