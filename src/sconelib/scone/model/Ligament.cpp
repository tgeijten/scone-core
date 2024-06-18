/*
** Ligament.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Ligament.h"

#include "Joint.h"
#include "Dof.h"
#include "Body.h"
#include "Model.h"
#include "scone/core/math.h"
#include "xo/container/container_tools.h"

#pragma warning( disable: 4355 )

namespace scone
{
	Ligament::Ligament( const Model& model ) {}

	Ligament::~Ligament() {}

	Real Ligament::GetNormalizedMomentArm( const Dof& dof ) const
	{
		Real mom = GetMomentArm( dof );
		if ( mom != 0 )
		{
			// normalize
			Real total_mom = 0.0;
			for ( auto& d : GetModel().GetDofs() )
				total_mom += abs( GetMomentArm( *d ) );
			return mom / total_mom;
		}
		else return mom;
	}

	Real Ligament::GetMoment( const Dof& dof ) const
	{
		return GetForce() * GetMomentArm( dof );
	}

	Side Ligament::GetSide() const
	{
		return GetSideFromName( GetName() );
	}

	bool Ligament::HasMomentArm( const Dof& dof ) const
	{
		for ( const auto& d : m_Dofs )
			if ( d == &dof )
				return true;
		return false;
	}

	const std::vector< const Joint* >& Ligament::GetJoints() const
	{
		SCONE_ASSERT( !m_Joints.empty() ); // Initialize in derived class via InitJointsDofs()
		return m_Joints;
	}

	const std::vector<const Dof*>& Ligament::GetDofs() const
	{
		return m_Dofs;
	}

	void Ligament::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		if ( flags( StoreDataTypes::MuscleProperties ) ) {
			const auto& name = GetName();

			// basic ligament properties
			frame[name + ".length_norm"] = GetNormalizedLength();
			frame[name + ".velocity_norm"] = GetNormalizedVelocity();
			frame[name + ".force_norm"] = GetNormalizedForce();

			// detailed ligament properties
			if ( flags( StoreDataTypes::MusclePropertiesDetailed ) ) {
				frame[name + ".length"] = GetLength();
				frame[name + ".velocity"] = GetVelocity();
				frame[name + ".force"] = GetForce();
				frame[name + ".power"] = GetForce() * GetVelocity();
			}
		}

		if ( flags( StoreDataTypes::MuscleDofMomentPower ) ) {
			for ( auto& d : GetDofs() ) {
				auto name = GetName() + "." + d->GetName();
				auto ma = GetMomentArm( *d );
				auto mom = GetForce() * ma;
				frame[name + ".moment_arm"] = ma;
				frame[name + ".moment"] = mom;
				frame[name + ".power"] = mom * d->GetVel();
			}
		}
	}

	PropNode Ligament::GetInfo() const
	{
		PropNode pn;
		pn["name"] = GetName();
		pn["origin"] = GetOriginBody().GetName();
		pn["insertion"] = GetInsertionBody().GetName();
		pn["pcsa_force"] = GetPcsaForce();
		pn["resting_length"] = GetRestingLength();
		return pn;
	}

	void Ligament::InitBodyJointDofs( const Body* b )
	{
		if ( auto* j = b->GetJoint() )
		{
			m_Joints.push_back( j );
			xo::append( m_Dofs, j->GetDofs() );
		}
	}

	void Ligament::InitJointsDofs()
	{
		SCONE_ASSERT( m_Joints.empty() && m_Dofs.empty() );
		const Body* const orgBody = &GetOriginBody();
		const Body* const insBody = &GetInsertionBody();
		const Body* const rootBody = GetModel().HasRootBody() ? &GetModel().GetRootBody() : nullptr;
		const Body* b = nullptr;

		// add joints, traversing from insertion to origin
		for ( b = insBody; b && b != orgBody && b != rootBody; b = b->GetParentBody() )
			InitBodyJointDofs( b );

		if ( b == rootBody )
		{
			// In this case, the muscle crosses the root body and
			// we need to add joints from insertion to root as well.
			for ( b = orgBody; b && b != rootBody; b = b->GetParentBody() )
				InitBodyJointDofs( b );
		}
	}
}
