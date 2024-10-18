/*
** Actuator.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Actuator.h"
#include <cmath>

namespace scone
{
	Actuator::Actuator() :
		m_ActuatorInput( 0.0 )
	{}

	Actuator::~Actuator()
	{}

	void Actuator::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		if ( flags( StoreDataTypes::ActuatorInput ) )
			frame[GetName() + ".input"] = GetInput();
	}

	PropNode Actuator::GetInfo() const
	{
		return PropNode();
	}
}
