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
#include "muscle_tools.h"
#include "model_tools.h"

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

	bool Ligament::ActsOnDof( const Dof& dof ) const
	{
		return xo::contains( m_Dofs, &dof );
	}

	bool Ligament::ActsOnJoint( const Joint& joint ) const
	{
		return xo::contains( m_Joints, &joint );
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
		pn.add_child( "path", GetPathInfo( GetLocalLigamentPath() ) );
		return pn;
	}

	void Ligament::InitJointsDofs()
	{
		SCONE_ASSERT( m_Joints.empty() && m_Dofs.empty() );
		auto jvec = GetConnectingJoints( &GetOriginBody(), &GetInsertionBody(), GetModel().TryGetRootBody() );
		for ( auto* j : jvec ) {
			m_Joints.push_back( j );
			xo::append( m_Dofs, j->GetDofs() );
		}
		SCONE_ASSERT( !m_Joints.empty() );
	}
}
