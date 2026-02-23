/*
** Leg.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Leg.h"
#include "Body.h"
#include "Joint.h"
#include "scone/core/profiler_config.h"
#include "scone/core/string_tools.h"
#include "Model.h"
#include "model_tools.h"
#include "scone/core/Log.h"
#include "xo/utility/memory_tools.h"

namespace scone
{
	Leg::Leg( Body& proximal, Body& foot, size_t index, Side side, size_t rank, const ContactForce* cf ) :
		m_Side( side ),
		m_Rank( rank ),
		m_Index( index ),
		m_Upper( proximal ),
		m_Foot( foot ),
		m_Base( xo::dereference_or_throw( proximal.GetParentBody(), proximal.GetName() + " is an upper leg body without a parent" ) ),
		m_Name( stringf( "leg%d", index ) + ( ( side == Side::Left ) ? "_l" : "_r" ) ),
		m_ContactForce( cf )
	{
		// measure length during construction, as it could be pose-dependent
		m_LegLength = MeasureLength();

		for ( const auto& b : m_Upper.GetModel().GetBodies() )
			if ( b->IsChildOf( m_Upper ) && b->HasContactGeometry() )
				m_ContactBodies.emplace_back( b );
		if ( m_ContactBodies.empty() )
			log::warning( "Leg with upper body " + m_Upper.GetName() + " has no contact geometry" );
	}

	Leg::~Leg()
	{
	}

	Vec3 Leg::GetContactForce() const
	{
		Vec3 f;
		if ( m_ContactForce )
			f = m_ContactForce->GetForce();
		else for ( auto& b : m_ContactBodies )
			f += b->GetContactForce();
		return f;
	}

	Vec3 Leg::GetGroundContactForce() const
	{
		Vec3 f;
		if ( m_ContactForce )
			f = m_ContactForce->GetForce();
		else for ( auto& b : m_ContactBodies )
			f += b->GetGroundContactForce();
		return f;
	}

	Vec3 Leg::GetContactMoment() const
	{
		Vec3 m;
		if ( m_ContactForce )
			m = m_ContactForce->GetMoment();
		else for ( auto& b : m_ContactBodies )
			m += b->GetContactMoment();
		return m;
	}

	Vec3 Leg::GetContactPos() const
	{
		return GetContactForceValue().point;
	}

	Vec3 Leg::GetRelFootPos() const
	{
		return GetFootBody().GetComPos() - GetBaseBody().GetComPos();
	}

	ForceAtPoint Leg::GetContactForceValue() const
	{
		ForceAtPoint v;
		if ( m_ContactForce )
			v = m_ContactForce->GetForceValue();
		else for ( auto& b : m_ContactBodies )
			v += b->GetContactForceValue();
		return v;
	}

	Real Leg::MeasureLength() const
	{
		// HACK: this uses body positions because we don't have access to joint positions in OpenSim
		// OpenSim: how can we get the actual position of a joint
		// add all distances from foot to upper, using body origins
		double d = 0.0;
		for ( const Body* body = &m_Foot; body && body->GetParentBody() && body != &m_Upper; body = body->GetParentBody() )
			d += length( body->GetOriginPos() - body->GetParentBody()->GetOriginPos() );

		return d;
	}

	Real Leg::GetLoad() const
	{
		return xo::length( GetGroundContactForce() ) / m_Upper.GetModel().GetBW();
	}
}
