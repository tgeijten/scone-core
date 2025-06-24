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
		range_count( 0 )
	{
		INIT_PROP( props, offset, Vec3::zero() );
		INIT_PROP( props, target_position, Vec3::zero() );
		INIT_PROP( props, target_orientation, Quat::identity() );
		INIT_PROP( props, direction, Vec3::zero() );
		INIT_PROP( props, relative_to_model_com, false );
		INIT_PROP( props, magnitude, direction.is_null() );
		INIT_PROP( props, scale, Vec3::one() );
		INIT_PROP( props, position, RangePenalty<Real>() );
		INIT_PROP( props, orientation, RangePenalty<Real>() );
		INIT_PROP( props, velocity, RangePenalty<Real>() );
		INIT_PROP( props, angular_velocity, RangePenalty<Real>() );
		INIT_PROP( props, acceleration, RangePenalty<Real>() );
		INIT_PROP( props, angular_acceleration, RangePenalty<Real>() );

		range_count = int( !position.IsNull() ) + int( !velocity.IsNull() ) + int( !acceleration.IsNull() );
	}

	double BodyMeasure::ComputeResult( const Model& model )
	{
		double penalty = 0.0;
		if ( !position.IsNull() )
		{
			penalty += position.GetResult();
			if ( range_count > 1 )
				report_.set( name_ + ".pos_penalty", stringf( "%g", position.GetResult() ) );
		}
		if ( !orientation.IsNull() )
		{
			penalty += orientation.GetResult();
			if ( range_count > 1 )
				report_.set( name_ + ".ori_penalty", stringf( "%g", orientation.GetResult() ) );
		}
		if ( !velocity.IsNull() )
		{
			penalty += velocity.GetResult();
			if ( range_count > 1 )
				report_.set( name_ + ".vel_penalty", stringf( "%g", velocity.GetResult() ) );
		}
		if ( !angular_velocity.IsNull() )
		{
			penalty += angular_velocity.GetResult();
			if ( range_count > 1 )
				report_.set( name_ + ".ang_vel_penalty", stringf( "%g", angular_velocity.GetResult() ) );
		}
		if ( !acceleration.IsNull() )
		{
			penalty += acceleration.GetResult();
			if ( range_count > 1 )
				report_.set( name_ + ".acc_penalty", stringf( "%g", acceleration.GetResult() ) );
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
		if ( !position.IsNull() )
			frame[name + ".pos_penalty"] = position.GetLatest();
		if ( !orientation.IsNull() )
			frame[name + ".ori_penalty"] = orientation.GetLatest();
		if ( !velocity.IsNull() )
			frame[name + ".vel_penalty"] = velocity.GetLatest();
		if ( !angular_velocity.IsNull() )
			frame[name + ".ang_vel_penalty"] = angular_velocity.GetLatest();
		if ( !acceleration.IsNull() )
			frame[name + ".acc_penalty"] = acceleration.GetLatest();
	}
}
