/*
** BodyOrientationReflex.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
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
#include "scone/core/string_tools.h"
#include "scone/model/model_tools.h"
#include "xo/system/version.h"

namespace scone
{
	BodyOrientationReflex::BodyOrientationReflex( const PropNode& pn, Params& par, Model& model, ReflexController& rc, const Location& loc ) :
		Reflex( pn, par, model, rc, loc ),
		INIT_MEMBER_REQUIRED( pn, source ),
		axis( normalized( pn.get<Vec3>( "axis", Vec3::unit_z() ) ) ),
		INIT_MEMBER( pn, axis_name, GetVectorIdentifier( axis ) ),
		INIT_MEMBER( pn, use_rotation_vector, model.scone_version < xo::version( 2, 0, 4, 2348 ) ),
		u_p(), u_v(),
		m_SourceBody( *FindByLocation( model.GetBodies(), source, loc ) ),
		m_DelayedPos( use_rotation_vector ?
			model.AcquireDelayedSensor< BodyOrientationSensor >( m_SourceBody, axis, axis_name, loc.side_ ) :
			model.AcquireDelayedSensor< BodyEulerOriSensor >( m_SourceBody, GetAxisIndex( axis ), loc.side_ ) ),
		m_DelayedVel( model.AcquireDelayedSensor< BodyAngularVelocitySensor >( m_SourceBody, axis, axis_name, loc.side_ ) )
	{
		String par_name = model.scone_version >= xo::version( 2, 0, 6 ) ? GetParName( pn, loc ) : target + '-' + GetNameNoSide( source );
		ScopedParamSetPrefixer prefixer( par, par_name + "." );

		INIT_PAR( pn, par, delay, 0.0 );

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
		frame[GetReflexName( actuator_.GetName(), m_DelayedPos.GetName() )] = u_p;
		frame[GetReflexName( actuator_.GetName(), m_DelayedVel.GetName() )] = u_v;
	}
}
