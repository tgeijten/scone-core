/*
** Dof.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Dof.h"
#include "Model.h"
#include "Muscle.h"
#include "Joint.h"

#pragma warning( disable: 4355 )

namespace scone
{
	Dof::Dof( const Joint* j ) : m_Joint( j ) {}
	Dof::~Dof() {}

	Real Dof::GetMuscleMoment() const
	{
		Real mom = 0.0;
		for ( const auto& mus : GetModel().GetMuscles() )
		{
			if ( mus->ActsOnDof( *this ) )
				mom += mus->GetMoment( *this );
		}
		return mom;
	}

	Side Dof::GetSide() const
	{
		return GetSideFromName( GetName() );
	}

	PropNode Dof::GetInfo() const
	{
		PropNode pn = Actuator::GetInfo();
		pn["name"] = GetName();
		if ( GetJoint() )
			pn["joint"] = GetJoint()->GetName();
		pn["type"] = IsRotational() ? "rotational" : "translational";
		Real f = IsRotational() ? xo::rad_to_deg( 1.0 ) : 1.0;
		pn["min"] = f * GetRange().min;
		pn["max"] = f * GetRange().max;
		if ( IsActuated() ) {
			pn["min_torque"] = GetMinTorque();
			pn["max_torque"] = GetMaxTorque();
		}
		if ( IsRotational() )
			pn["rotation_axis"] = GetRotationAxis();
		return pn;
	}
}
