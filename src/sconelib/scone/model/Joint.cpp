/*
** Joint.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Joint.h"
#include "Body.h"
#include "Model.h"
#include "Dof.h"

namespace scone
{
	Joint::Joint( Body& body, Body& parent_body ) :
		m_Body( body ),
		m_ParentBody( parent_body )
	{
		if ( m_Body.GetMass() != 0 )
			m_Body.m_Joint = this;
	}

	Vec3 Joint::GetLimitTorque() const
	{
		auto torque = Vec3::zero();
		for ( const auto& d : GetDofs() )
			torque += d->GetLimitMoment() * d->GetRotationAxis();
		return torque;
	}

	Real Joint::GetLimitPower() const
	{
		Real pow = 0.0;
		for ( const auto& d : GetDofs() )
			pow += d->GetVel() * d->GetLimitMoment();
		return pow;
	}

	Real Joint::GetLoad() const
	{
		return xo::length( GetReactionForce() ) / m_Body.GetModel().GetBW();
	}

	const std::vector< Dof* >& Joint::GetDofs() const
	{
		if ( m_Dofs.empty() )
		{
			for ( auto& dof : m_Body.GetModel().GetDofs() )
				if ( this == dof->GetJoint() )
					m_Dofs.push_back( dof );
		}
		return m_Dofs;
	}

	void Joint::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		// store joint reaction force magnitude
		if ( flags( StoreDataTypes::JointReactionForce ) )
			frame[GetName() + ".load"] = GetLoad();
	}

	PropNode Joint::GetInfo() const
	{
		PropNode pn;
		pn["name"] = GetName();
		pn["parent"] = GetParentBody().GetName();
		pn["child"] = GetBody().GetName();
		pn["pos_in_parent"] = GetPosInParent();
		pn["pos_in_child"] = GetPosInChild();
		if ( !m_Dofs.empty() ) {
			auto& dof_pn = pn["coordinates"];
			for ( auto* d : m_Dofs )
				dof_pn.add_value( d->GetName() );
		}
		pn["pos_in_child"] = GetPosInChild();
		return pn;
	}

	Joint::~Joint()
	{}
}
