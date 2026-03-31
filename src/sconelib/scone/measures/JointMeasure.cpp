/*
** JointMeasure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "JointMeasure.h"
#include "scone/model/Model.h"
#include "scone/core/string_tools.h"
#include "xo/geometry/vec3.h"
#include "xo/container/container_algorithms.h"

namespace scone
{
	JointMeasure::JointMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
		Measure( props, par, model, loc ),
		joint( *FindByLocation( model.GetJoints(), props.get< String >( "joint" ), loc ) ),
		INIT_MEMBER( props, joint_force, RangePenalty<Real>() ),
		INIT_MEMBER( props, limit_torque, RangePenalty<Real>() ),
		INIT_MEMBER( props, muscle_torque, RangePenalty<Real>() ),
		INIT_MEMBER( props, motor_torque, RangePenalty<Real>() ),
		penalties{ {
			{ &joint_force, "joint_force" },
			{ &limit_torque, "limit_torque" },
			{ &muscle_torque, "muscle_torque" },
			{ &motor_torque, "motor_torque" } } }
	{
		if ( name_.empty() )
			name_ = joint.GetName();
	}

	double JointMeasure::ComputeResult( const Model& model )
	{
		double penalty = 0.0;
		auto range_count = xo::count_if( penalties, [&]( auto&& p ) { return !p.first->IsNull(); } );
		for ( const auto& [p, name] : penalties ) {
			if ( !p->IsNull() ) {
				penalty += p->GetResult();
				if ( range_count > 1 )
					report_.set( name_ + "." + name, stringf( "%g", p->GetResult() ) );
			}
		}

		return  penalty;
	}

	double JointMeasure::GetCurrentResult( const Model& model )
	{
		return xo::accumulate( penalties, 0.0, [&]( auto v, auto&& p ) { return v + p.first->GetLatest(); } );
	}

	void JointMeasure::Reset( Model& model )
	{
		Measure::Reset( model );
		for ( const auto& [p, name] : penalties )
			p->Reset();
	}

	UpdateResult JointMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		if ( !joint_force.IsNull() )
			joint_force.AddSample( timestamp, xo::length( joint.GetReactionForce() ) );
		if ( !limit_torque.IsNull() )
			limit_torque.AddSample( timestamp, xo::length( joint.GetLimitTorque() ) );
		if ( !muscle_torque.IsNull() )
			muscle_torque.AddSample( timestamp, xo::length( joint.GetMuscleMoment() ) );
		if ( !motor_torque.IsNull() )
			motor_torque.AddSample( timestamp, xo::length( joint.GetMotorTorque() ) );

		return false;
	}

	String JointMeasure::GetClassSignature() const
	{
		return String();
	}

	void JointMeasure::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		for ( const auto& [p, name] : penalties )
			if ( !p->IsNull() )
				frame[joint.GetName() + "." + name + "_penalty"] = p->GetLatest();
	}
}
