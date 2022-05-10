/*
** BodyOrientationReflex.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "BodyOrientationReflex.h"
#include "scone/model/Side.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/Model.h"
#include "scone/model/Sensors.h"
#include "scone/model/SensorDelayAdapter.h"
#include "scone/model/Actuator.h"

namespace scone
{
	BodyOrientationReflex::BodyOrientationReflex( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Reflex( pn, par, model, loc ),
		INIT_MEMBER_REQUIRED( pn, source ),
		INIT_MEMBER( pn, axis, Vec3::unit_z() ),
		INIT_MEMBER( pn, axis_name, "" ),
		INIT_MEMBER( pn, mirror_left, false ),
		m_Mirror( mirror_left && loc.side_ == Side::Left ),
		m_SourceBody( *FindByNameTrySided( model.GetBodies(), source, loc.side_ ) ),
		m_DelayedPos( model.AcquireDelayedSensor< BodyOrientationSensor >( m_SourceBody, axis, axis_name, loc.side_ ) ),
		m_DelayedVel( model.AcquireDelayedSensor< BodyAngularVelocitySensor >( m_SourceBody, axis, axis_name, loc.side_ ) )
	{
		String par_name = GetParName( pn, loc );
		ScopedParamSetPrefixer prefixer( par, par_name + "." );

		INIT_PAR_NAMED( pn, par, P0, "P0", 0.0 );
		INIT_PAR_NAMED( pn, par, KP, "KP", 0.0 );
		INIT_PROP( pn, allow_neg_P, true );

		INIT_PAR_NAMED( pn, par, V0, "V0", 0.0 );
		INIT_PAR_NAMED( pn, par, KV, "KV", 0.0 );
		INIT_PROP( pn, allow_neg_V, true );

		INIT_PAR_NAMED( pn, par, C0, "C0", 0.0 );
	}

	BodyOrientationReflex::~BodyOrientationReflex()
	{}

	void BodyOrientationReflex::ComputeControls( double timestamp )
	{
		Real pos = m_DelayedPos.GetValue( delay );
		Real vel = m_DelayedVel.GetValue( delay );

		auto delta_pos = P0 - pos;
		auto delta_vel = V0 - vel;

		if ( m_Mirror ) {
			delta_pos = -delta_pos;
			delta_vel= -delta_vel;
		}

		u_p = KP * delta_pos;
		if ( !allow_neg_P && u_p < 0.0 )
			u_p = 0.0;

		u_v = KV * delta_vel;
		if ( !allow_neg_V && u_v < 0.0 )
			u_v = 0.0;

		AddTargetControlValue( C0 + u_p + u_v );
	}

	void BodyOrientationReflex::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		auto name = GetReflexName( actuator_.GetName(), source );
		frame[ name + ".RBP" ] = u_p;
		frame[ name + ".RBV" ] = u_v;
	}
}
