/*
** MuscleReflex.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "MuscleReflex.h"
#include "scone/model/Muscle.h"
#include "scone/model/Location.h"
#include "scone/model/Dof.h"
#include "scone/model/Sensors.h"
#include "scone/model/Model.h"

namespace scone
{
	MuscleReflex::MuscleReflex( const PropNode& pn, Params& par, Model& model, ReflexController& rc, const Location& loc ) :
		Reflex( pn, par, model, rc, loc ),
		//source( *FindByLocation( model.GetMuscles(), pn.get<string>( "source", target ), loc ) ),
		source( model.FindMuscleOrGroupByLocation( pn.get<string>( "source", target ), loc ) ),
		m_pForceSensor( nullptr ),
		m_pLengthSensor( nullptr ),
		m_pVelocitySensor( nullptr ),
		m_pSpindleSensor( nullptr ),
		m_pActivationSensor( nullptr )
	{
		// init names
		String par_name = GetParName( pn, loc );
		ScopedParamSetPrefixer prefixer( par, par_name + "." );
		String control_name = par.prefix();

		INIT_PAR( pn, par, delay, 0.0 );

		INIT_PAR( pn, par, KL, 0.0 );
		INIT_PAR( pn, par, L0, 1.0 );
		INIT_PROP( pn, allow_neg_L, true );

		INIT_PAR( pn, par, KV, 0.0 );
		INIT_PAR( pn, par, V0, 0.0 );
		INIT_PROP( pn, allow_neg_V, false );

		INIT_PAR( pn, par, KF, 0.0 );
		INIT_PAR( pn, par, F0, 0.0 );
		INIT_PROP( pn, allow_neg_F, true );

		INIT_PAR( pn, par, KS, 0.0 );
		INIT_PAR( pn, par, S0, 0.0 );
		INIT_PROP( pn, allow_neg_S, false );

		INIT_PAR( pn, par, KA, 0.0 );
		INIT_PAR( pn, par, A0, 0.0 );
		INIT_PROP( pn, allow_neg_A, false );

		INIT_PAR( pn, par, C0, 0.0 );

		// create delayed sensors
		controls_.Add( control_name + "C0", &C0 );

		if ( KF != 0.0 ) {
			m_pForceSensor = &model.AcquireDelayedSensor< MuscleForceSensor >( source );
			controls_.Add( control_name + "KF", &KF );
			controls_.Add( control_name + "F0", &F0 );
		}

		if ( KL != 0.0 ) {
			m_pLengthSensor = &model.AcquireDelayedSensor< MuscleLengthSensor >( source );
			controls_.Add( control_name + "KL", &KL );
			controls_.Add( control_name + "L0", &L0 );
		}

		if ( KV != 0.0 ) {
			m_pVelocitySensor = &model.AcquireDelayedSensor< MuscleVelocitySensor >( source );
			controls_.Add( control_name + "KV", &KV );
			controls_.Add( control_name + "V0", &V0 );
		}

		if ( KS != 0.0 ) {
			m_pSpindleSensor = &model.AcquireDelayedSensor< MuscleSpindleSensor >( source );
			controls_.Add( control_name + "KS", &KS );
			controls_.Add( control_name + "S0", &S0 );
		}

		if ( KA != 0.0 ) {
			m_pActivationSensor = &model.AcquireDelayedSensor< MuscleActivationSensor >( source );
			controls_.Add( control_name + "KA", &KA );
			controls_.Add( control_name + "A0", &A0 );
		}

		//log::TraceF( "MuscleReflex SRC=%s TRG=%s KL=%.2f KF=%.2f C0=%.2f", source.GetName().c_str(), m_Target.GetName().c_str(), length_gain, force_gain, u_constant );
	}

	MuscleReflex::~MuscleReflex()
	{
	}

	void MuscleReflex::ComputeControls( double timestamp )
	{
		// add stretch reflex
		u_l = GetValue( m_pLengthSensor, KL, L0, allow_neg_L );

		// add velocity reflex
		u_v = GetValue( m_pVelocitySensor, KV, V0, allow_neg_V );

		// add force reflex
		u_f = GetValue( m_pForceSensor, KF, F0, allow_neg_F );

		// add spindle reflex
		u_s = GetValue( m_pSpindleSensor, KS, S0, allow_neg_S );

		// add spindle reflex
		u_a = GetValue( m_pActivationSensor, KA, A0, allow_neg_A );

		// sum it up
		u_total = u_l + u_v + u_f + u_s + u_a + C0;
		AddTargetControlValue( u_total );
	}

	void MuscleReflex::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		auto name = GetReflexName( actuator_.GetName(), source.GetName() );

		if ( m_pLengthSensor )
			frame[name + ".RL"] = u_l;
		if ( m_pVelocitySensor )
			frame[name + ".RV"] = u_v;
		if ( m_pForceSensor )
			frame[name + ".RF"] = u_f;
		if ( m_pSpindleSensor )
			frame[name + ".RS"] = u_s;
		if ( m_pActivationSensor )
			frame[name + ".RA"] = u_a;
		//frame[ name + ".R" ] = u_total;
	}
}
