/*
** SensorReflex.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "SensorReflex.h"
#include "scone/model/SensorDelayAdapter.h"
#include "scone/model/Actuator.h"

namespace scone
{
	SensorReflex::SensorReflex( const PropNode& pn, Params& par, Model& model, ReflexController& rc, const Location& loc, SensorDelayAdapter& s ) :
		Reflex( pn, par, model, rc, loc ),
		INIT_MEMBER( pn, mirror_left, false ),
		val_(),
		sign_( mirror_left&& loc.GetSide() == Side::Left ? -1.0 : 1.0 ),
		source_( s )
	{
		String par_name = GetParName( pn, loc );
		ScopedParamSetPrefixer prefixer( par, par_name + "." );

		INIT_PAR( pn, par, delay, 0.0 );

		INIT_PAR_NAMED( pn, par, P0, "P0", 0.0 );
		INIT_PAR_NAMED( pn, par, KP, "KP", 0.0 );
		INIT_PROP( pn, allow_neg_P, true );

		INIT_PAR_NAMED( pn, par, C0, "C0", 0.0 );
	}

	void SensorReflex::ComputeControls( double timestamp )
	{
		Real pos = sign_ * source_.GetValue( delay );
		val_ = KP * pos - P0;

		if ( !allow_neg_P && val_ < 0.0 )
			val_ = 0.0;

		AddTargetControlValue( C0 + val_ );
	}

	void SensorReflex::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		auto name = GetReflexName( actuator_.GetName(), source_.GetName() );
		frame[name + ".V"] = val_;
	}
}
