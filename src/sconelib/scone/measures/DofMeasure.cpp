/*
** DofMeasure.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "DofMeasure.h"
#include "scone/model/Model.h"
#include "scone/core/string_tools.h"

namespace scone
{
	DofMeasure::DofMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
	Measure( props, par, model, loc ),
	dof( *FindByLocation( model.GetDofs(), props.get< String >( "dof" ), loc ) ),
	parent( nullptr ),
	range_count( 0 )
	{
		if ( props.try_get< String >( "parent" ) )
			parent = FindByLocation( model.GetDofs(), props.get< String >( "parent" ), loc );

		INIT_PROP( props, position, RangePenalty<Degree>() );
		INIT_PROP( props, velocity, RangePenalty<Degree>() );
		INIT_PROP( props, acceleration, RangePenalty<Degree>() );
		limit_torque = props.get_any<RangePenalty<Real>>( { "force", "limit_torque" }, RangePenalty<Real>() );
		INIT_PROP( props, actuator_torque, RangePenalty<Real>() );

		range_count = int( !position.IsNull() ) + int( !velocity.IsNull() ) + int( !acceleration.IsNull() ) + int( !limit_torque.IsNull() );
		if ( name_.empty() )
			name_ = dof.GetName();
	}

	double DofMeasure::ComputeResult( const Model& model )
	{
		double penalty = 0;
		if ( !position.IsNull() )
		{
			penalty += position.GetResult().value;
			if ( range_count > 1 )
				GetReport().set( name_ + ".position_penalty" , stringf( "%g", position.GetResult() ) );
		}
		if ( !velocity.IsNull() )
		{
			penalty += velocity.GetResult().value;
			if ( range_count > 1 )
				GetReport().set( name_ + ".velocity_penalty", stringf( "%g", velocity.GetResult() ) );
		}
		if ( !acceleration.IsNull() )
		{
			penalty += acceleration.GetResult().value;
			if ( range_count > 1 )
				GetReport().set( name_ + ".acceleration_penalty", stringf( "%g", acceleration.GetResult() ) );
		}
		if ( !limit_torque.IsNull() )
		{
			penalty += limit_torque.GetResult();
			if ( range_count > 1 )
				GetReport().set( name_ + ".limit_torque_penalty", stringf( "%g", limit_torque.GetResult() ) );
		}
		if ( !actuator_torque.IsNull() )
		{
			penalty += actuator_torque.GetResult();
			if ( range_count > 1 )
				GetReport().set( name_ + ".actuator_torque_penalty", stringf( "%g", actuator_torque.GetResult() ) );
		}

		return penalty;
	}

	double DofMeasure::GetCurrentResult( const Model& model )
	{
		return position.GetLatest().value + velocity.GetLatest().value + acceleration.GetLatest().value + 
			limit_torque.GetLatest() + actuator_torque.GetLatest();
	}

	void DofMeasure::Reset( Model& model )
	{
		Measure::Reset( model );
		position.Reset(); velocity.Reset(); acceleration.Reset(); limit_torque.Reset(); actuator_torque.Reset();
	}

	bool DofMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		if ( !position.IsNull() )
			position.AddSample( timestamp, Degree( Radian( dof.GetPos() + ( parent ? parent->GetPos() : 0 ) ) ) );
		if ( !velocity.IsNull() )
			velocity.AddSample( timestamp, Degree( Radian( dof.GetVel() + ( parent ? parent->GetVel() : 0 ) ) ) );
		if ( !acceleration.IsNull() )
			acceleration.AddSample( timestamp, Degree( Radian( dof.GetAcc() + ( parent ? parent->GetAcc() : 0 ) ) ) );
		if ( !limit_torque.IsNull() )
			limit_torque.AddSample( timestamp, dof.GetLimitMoment() );
		if ( !actuator_torque.IsNull() )
			actuator_torque.AddSample( timestamp, dof.GetActuatorTorque() );
		return false;
	}

	String DofMeasure::GetClassSignature() const
	{
		return String();
	}

	void DofMeasure::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		if ( !position.IsNull() )
			frame[ dof.GetName() + ".position_penalty" ] = position.GetLatest().value;
		if ( !velocity.IsNull() )
			frame[ dof.GetName() + ".velocity_penalty" ] = velocity.GetLatest().value;
		if ( !acceleration.IsNull() )
			frame[ dof.GetName() + ".acceleration_penalty" ] = acceleration.GetLatest().value;
		if ( !limit_torque.IsNull() )
			frame[ dof.GetName() + ".limit_torque_penalty" ] = limit_torque.GetLatest();
		if ( !actuator_torque.IsNull() )
			frame[ dof.GetName() + ".actuator_torque_penalty" ] = actuator_torque.GetLatest();
	}
}
