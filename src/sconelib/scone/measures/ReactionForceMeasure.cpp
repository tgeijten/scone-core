/*
** ReactionForceMeasure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "ReactionForceMeasure.h"

#include "scone/core/HasName.h"
#include "scone/model/Model.h"
#include "xo/geometry/vec3.h"
#include "xo/container/zip.h"

namespace scone
{
	ReactionForceMeasure::ReactionForceMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
		Measure( props, par, model, loc ),
		RangePenalty<Real>( props ),
		INIT_MEMBER( props, use_force_per_leg, false ),
		INIT_MEMBER( props, velocity, {} ),
		INIT_MEMBER( props, acceleration, {} )
	{
		if ( name_.empty() )
			name_ = "grf";

		for ( auto& leg : model.GetLegs() )
			derivatives_.push_back( { leg.GetName(), {} } );
	}

	double ReactionForceMeasure::ComputeResult( const Model& model )
	{
		double result = RangePenalty<Real>::GetResult();
		result += velocity.GetResult();
		result += acceleration.GetResult();

		return result;
	}

	double ReactionForceMeasure::GetCurrentResult( const Model& model )
	{
		double result = RangePenalty<Real>::GetLatest();
		result += velocity.GetLatest();
		result += acceleration.GetLatest();

		return result;
	}

	void ReactionForceMeasure::Reset( Model& model )
	{
		Measure::Reset( model );
		RangePenalty<Real>::Reset();
		velocity.Reset();
		acceleration.Reset();
	}

	UpdateResult ReactionForceMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		Real leg_load = 0.0f;
		if ( use_force_per_leg ) {
			for ( auto& leg : model.GetLegs() )
				leg_load = xo::max( leg_load, xo::length( leg.GetContactForce() ) / model.GetBW() );
		} else {
			for ( auto& leg : model.GetLegs() )
				leg_load += xo::length( leg.GetContactForce() ) / model.GetBW();
		}

		AddSample( timestamp, leg_load );

		for ( auto& [leg, d] : xo::zip( model.GetLegs(), derivatives_ ) ) {
			d.second.update( leg.GetLoad(), timestamp );
			if ( !velocity.IsNull() )
				velocity.AddSample( d.second.velocity() );
			if ( !acceleration.IsNull() )
				acceleration.AddSample( d.second.acceleration() );
		}

		return false;
	}

	void ReactionForceMeasure::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		frame[name_ + ".load_penalty"] = GetLatest();
		if ( !velocity.IsNull() )
			frame[name_ + ".velocity_penalty"] = velocity.GetLatest();
		if ( !acceleration.IsNull() )
			frame[name_ + ".acceleration_penalty"] = acceleration.GetLatest();

		for ( auto& d : derivatives_ ) {
			auto str = name_ + "." + d.first;
			frame[str + ".load"] = d.second.value();
			if ( !velocity.IsNull() )
				frame[str + ".load_velocity"] = d.second.velocity();
			if ( !acceleration.IsNull() )
				frame[str + ".load_acceleration"] = d.second.acceleration();
		}
	}
}
