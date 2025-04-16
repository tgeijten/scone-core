/*
** BodyPostureMuscleReflex.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "BodyPostureMuscleReflex.h"
#include "scone/model/Side.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/Model.h"
#include "scone/model/Muscle.h"
#include "scone/model/Joint.h"
#include "scone/model/Sensors.h"
#include "scone/model/SensorDelayAdapter.h"
#include "scone/model/Actuator.h"
#include "scone/core/string_tools.h"
#include "scone/model/model_tools.h"

namespace scone
{
	BodyPostureMuscleReflex::BodyPostureMuscleReflex( const PropNode& pn, Params& par, Model& model, ReflexController& rc, const Location& loc ) :
		Reflex( pn, par, model, rc, loc ),
		INIT_MEMBER_REQUIRED( pn, source ),
		INIT_MEMBER( pn, joint, "" ),
		u_(),
		m_SourceBody( *FindByLocation( model.GetBodies(), source, loc ) )
	{
		ScopedParamSetPrefixer prefixer( par, GetParName( pn, loc ) + "." );

		target_orientation = try_get_par( par, "target_orientation", pn, Vec3Deg::zero() );

		INIT_PAR( pn, par, delay, 0.0 );
		INIT_PAR_NAMED( pn, par, KP, "KP", 0.0 );
		INIT_PAR_NAMED( pn, par, KV, "KV", 0.0 );
		INIT_PAR_NAMED( pn, par, C0, "C0", 0.0 );

		auto& mus = *FindByLocation( model.GetMuscles(), target, loc );
		Joint* jp = nullptr;
		if ( !joint.empty() )
			jp = FindByNameTrySided( model.GetJoints(), joint, loc.side_ );
		auto q = quat_from_euler_yzx( target_orientation );

		m_BodyPostureMuscleSensor = &model.AcquireDelayedSensor<BodyPostureMuscleSensor>( m_SourceBody, mus, jp, q, KP, KV );
	}

	BodyPostureMuscleReflex::~BodyPostureMuscleReflex()
	{}

	void BodyPostureMuscleReflex::ComputeControls( double timestamp )
	{
		u_ = m_BodyPostureMuscleSensor->GetValue( delay );
		AddTargetControlValue( C0 + u_ );
	}

	void BodyPostureMuscleReflex::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		frame[GetReflexName( actuator_.GetName(), m_BodyPostureMuscleSensor->GetName() )] = u_;
	}
}
