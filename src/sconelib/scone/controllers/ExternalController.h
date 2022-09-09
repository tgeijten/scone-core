/*
** ExternalController.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once
#include "Controller.h"
#include "scone/model/DelayBuffer.h"

namespace scone
{
	/// Controller with inputs that are externally set
	class SCONE_API ExternalController : public Controller
	{
	public:
		ExternalController( const PropNode& pn, Params& par, Model& model, const Location& loc );
		virtual ~ExternalController() = default;

		// set the input for actuator idx
		void SetInput( index_t idx, Real input );

		// set the input for actuator idx
		void SetDelayedInput( index_t idx, Real input );

		size_t GetActuatorCount() { return actuator_count_; }

		void Reset( Model& model ) override;

	protected:
		virtual bool ComputeControls( Model& model, double timestamp ) override;
		virtual String GetClassSignature() const override;

		size_t actuator_count_;
		std::vector<Real> inputs_;
		std::vector<Real> delayed_inputs_;
		std::vector<DelayedActuatorValue> delayed_actuators_;
	};
}
