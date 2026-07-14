/*
** BodyMeasure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "BodyMeasure.h"
#include "scone/model/Model.h"
#include "scone/core/string_tools.h"

namespace scone
{
	BodyMeasure::BodyMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
		Measure( props, par, model, loc ),
		body( *FindByLocation( model.GetBodies(), props.get< String >( "body" ), loc ) ),
		INIT_MEMBER( props, target_position, Vec3::zero() ),
		INIT_MEMBER( props, target_orientation, Quat::identity() ),
		INIT_MEMBER( props, offset, Vec3::zero() ),
		INIT_MEMBER( props, direction, Vec3::zero() ),
		INIT_MEMBER( props, scale, Vec3::one() ),
		INIT_MEMBER( props, magnitude, direction.is_null() ),
		INIT_MEMBER( props, relative_to_model_com, false ),
		INIT_MEMBER( props, use_local_direction, false ),
		INIT_MEMBER( props, position, RangePenalty<Real>() ),
		INIT_MEMBER( props, orientation, RangePenalty<Real>() ),
		INIT_MEMBER( props, velocity, RangePenalty<Real>() ),
		INIT_MEMBER( props, angular_velocity, RangePenalty<Real>() ),
		INIT_MEMBER( props, acceleration, RangePenalty<Real>() ),
		INIT_MEMBER( props, angular_acceleration, RangePenalty<Real>() ),
		penalty_count( 0 ),
		penalties_{
			{ &position, "pos" },
			{ &orientation, "ori" },
			{ &velocity, "vel" },
			{ &angular_velocity, "ang_vel" },
			{ &acceleration, "acc" },
			{ &angular_acceleration, "ang_acc" },
		}
	{
		penalty_count = xo::count_if( penalties_, [&]( auto&& p ) { return !p.first->IsNull(); } );
		SCONE_THROW_IF( penalty_count == 0, "No penalties defined in BodyMeasure" );
		if ( name_.empty() )
			name_ = body.GetName();
	}

	double BodyMeasure::ComputeResult( const Model& model )
	{
		double penalty = 0.0;
		for ( const auto& [pen, name] : penalties_ ) {
			penalty += pen->GetResult();
			if ( penalty_count > 1 )
				report_.set( name_ + "." + name + "_penalty", stringf( "%g", pen->GetResult() ) );
		}

		return  penalty;
	}

	double BodyMeasure::GetCurrentResult( const Model& model )
	{
		return position.GetLatest() + velocity.GetLatest() + angular_velocity.GetLatest() + acceleration.GetLatest();
	}

	void BodyMeasure::Reset( Model& model )
	{
		Measure::Reset( model );
		position.Reset(); velocity.Reset(); angular_velocity.Reset(); acceleration.Reset();
	}

	UpdateResult BodyMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		if ( !position.IsNull() )
		{
			auto pos = body.GetPosOfPointOnBody( offset );
			if ( relative_to_model_com ) pos -= model.GetComPos();
			if ( !target_position.is_null() ) pos -= target_position;
			position.AddSample( timestamp, GetPenaltyValue( pos ) );
		}

		if ( !orientation.IsNull() )
		{
			auto ori = body.GetOrientation();
			auto delta = -ori * target_orientation;
			auto rot_vec = xo::rotation_vector_from_quat( delta );
			orientation.AddSample( timestamp, GetPenaltyValue( rot_vec ) );
		}

		if ( !velocity.IsNull() )
		{
			auto vel = body.GetLinVelOfPointOnBody( offset );
			if ( relative_to_model_com ) vel -= model.GetComVel();
			velocity.AddSample( timestamp, GetPenaltyValue( vel ) );
		}

		if ( !angular_velocity.IsNull() )
		{
			auto vel = body.GetAngVel();
			angular_velocity.AddSample( timestamp, GetPenaltyValue( vel ) );
		}

		if ( !acceleration.IsNull() )
		{
			auto acc = body.GetLinAccOfPointOnBody( offset );
			if ( relative_to_model_com ) acc -= model.GetComAcc();
			acceleration.AddSample( timestamp, GetPenaltyValue( acc ) );
		}

		if ( !angular_acceleration.IsNull() )
		{
			auto acc = body.GetAngAcc();
			angular_acceleration.AddSample( timestamp, GetPenaltyValue( acc ) );
		}

		return false;
	}

	String BodyMeasure::GetClassSignature() const
	{
		return String();
	}

	void BodyMeasure::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		String name = GetName().empty() ? body.GetName() : GetName() + "." + body.GetName();
		for ( const auto& [pen, pen_name] : penalties_ ) {
			if ( !pen->IsNull() )
				frame[name + '.' + pen_name + "_penalty"] = pen->GetLatest();
		}
	}

	Real BodyMeasure::GetPenaltyValue( const Vec3 v ) const
	{
		if ( magnitude )
			return length( scaled( v, scale ) );
		else if ( use_local_direction )
			return dot_product( body.GetOrientation() * direction, v );
		else return dot_product( direction, v );
	}
}
