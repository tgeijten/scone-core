/*
** Actuator.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/core/HasName.h"
#include "scone/core/HasData.h"
#include "scone/core/Storage.h"
#include "scone/core/PropNode.h"
#include "xo/container/circular_deque.h"

namespace scone
{
	class SCONE_API Actuator : public HasName, public HasData
	{
	public:
		Actuator();
		virtual ~Actuator();

		virtual double AddInput( double v ) { return m_ActuatorInput += v; }

		double GetInput() const { return m_ActuatorInput; }
		virtual double GetAdaptedInput() const { return xo::clamped( GetInput(), GetMinInput(), GetMaxInput() ); }

		virtual Real GetMinInput() const = 0;
		virtual Real GetMaxInput() const = 0;

		virtual void StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const override;
		virtual PropNode GetInfo() const;

		virtual void ClearInput() { m_ActuatorInput = 0.0; }

	protected:
		double m_ActuatorInput;
	};
}
