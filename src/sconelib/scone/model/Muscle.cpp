/*
** Muscle.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Muscle.h"

#include "Joint.h"
#include "Dof.h"
#include "xo/numerical/math.h"
#include "Body.h"
#include "Model.h"
#include "scone/core/math.h"
#include "scone/core/profiler_config.h"
#include "muscle_tools.h"

#pragma warning( disable: 4355 )

namespace scone
{
	Muscle::Muscle( const Model& model ) :
		Actuator(),
		m_MinActivation( model.min_muscle_activation ),
		m_MaxActivation( model.max_muscle_activation )
	{}

	Muscle::~Muscle()
	{}

	double Muscle::GetAdaptedInput() const
	{
		const auto& sb = GetModel().muscle_input_soft_limits;
		if ( sb.first > 0.0 && sb.second < 1.0 )
			return GetSoftLimitInput( sb.first, sb.second );
		else return GetClampedInput();
	}

	Real Muscle::GetNormalizedMomentArm( const Dof& dof ) const
	{
		Real mom = GetMomentArm( dof );
		if ( mom != 0 )
		{
			// normalize
			Real total_mom = 0.0;
			for ( auto& d : GetModel().GetDofs() )
				total_mom += abs( GetMomentArm( *d ) );
			return mom / total_mom;
		}
		else return mom;
	}

	Real Muscle::GetMoment( const Dof& dof ) const
	{
		return GetForce() * GetMomentArm( dof );
	}

	Vec3 Muscle::GetMoment3D( const Joint& j ) const
	{
		return GetForce() * GetMomentArm3D( j );
	}

	Real Muscle::GetMass( Real specific_tension, Real muscle_density ) const
	{
		// from OpenSim Umberger metabolic energy model docs
		return ( GetMaxIsometricForce() / specific_tension ) * muscle_density * GetOptimalFiberLength();
	}

	Real Muscle::GetPCSA( Real specific_tension ) const
	{
		return GetMaxIsometricForce() / specific_tension;
	}

	Real Muscle::GetVolume( Real specific_tension ) const
	{
		return GetPCSA( specific_tension ) * GetOptimalFiberLength();
	}

	Real Muscle::GetNormalizedSpindleRate() const
	{
		// derived from [Prochazka1999], velocity component normalized to unit length
		double vel = ( 65.0 / 200.0 ) * xo::signed_sqrt( GetNormalizedFiberVelocity() );
		double disp = GetNormalizedFiberLength();
		return std::max( 0.0, vel + disp );
	}

	Side Muscle::GetSide() const
	{
		return GetSideFromName( GetName() );
	}

	bool Muscle::ActsOnDof( const Dof& dof ) const
	{
		for ( const auto& d : m_Dofs )
			if ( d == &dof )
				return true;
		return false;
	}

	bool Muscle::ActsOnJoint( const Joint& joint ) const
	{
		for ( const auto& j : m_Joints )
			if ( j == &joint )
				return true;
		return false;
	}

	const std::vector<const Joint*>& Muscle::GetJoints() const
	{
		SCONE_ASSERT( !m_Joints.empty() ); // Initialize in derived class via InitJointsDofs()
		return m_Joints;
	}

	const std::vector<const Dof*>& Muscle::GetDofs() const
	{
		return m_Dofs;
	}

	bool Muscle::IsAntagonist( const Muscle& other ) const
	{
		for ( auto& dof : GetModel().GetDofs() )
		{
			if ( ActsOnDof( *dof ) && other.ActsOnDof( *dof ) )
				if ( Sign( GetMomentArm( *dof ) ) != Sign( other.GetMomentArm( *dof ) ) )
					return true;
		}
		return false;
	}

	bool Muscle::IsAgonist( const Muscle& other ) const
	{
		for ( auto& dof : GetModel().GetDofs() )
		{
			if ( ActsOnDof( *dof ) && other.ActsOnDof( *dof ) )
				if ( Sign( GetMomentArm( *dof ) ) == Sign( other.GetMomentArm( *dof ) ) )
					return true;
		}
		return false;
	}

	bool Muscle::HasSharedDofs( const Muscle& other ) const
	{
		for ( auto& dof : GetOriginBody().GetModel().GetDofs() )
		{
			if ( ActsOnDof( *dof ) && other.ActsOnDof( *dof ) )
				return true;
		}
		return false;
	}

	bool Muscle::HasSharedBodies( const Muscle& other ) const
	{
		return &GetOriginBody() == &other.GetOriginBody()
			|| &GetOriginBody() == &other.GetInsertionBody()
			|| &GetInsertionBody() == &other.GetOriginBody()
			|| &GetInsertionBody() == &other.GetInsertionBody();
	}

	bool Muscle::HasSharedJoints( const Muscle& other ) const
	{
		const Body* org1 = &GetOriginBody();
		const Body* org2 = &other.GetOriginBody();
		for ( const Body* b1 = &GetInsertionBody(); b1 != org1; b1 = b1->GetParentBody() )
			for ( const Body* b2 = &other.GetInsertionBody(); b2 != org2; b2 = b2->GetParentBody() )
				if ( b1->GetJoint() == b2->GetJoint() )
					return true;
		return false;
	}

	void Muscle::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		Actuator::StoreData( frame, flags );

		if ( flags( StoreDataTypes::ActuatorInput ) || flags( StoreDataTypes::MuscleProperties ) )
			frame[GetName() + ".excitation"] = GetExcitation();

		if ( flags( StoreDataTypes::MuscleProperties ) || flags( StoreDataTypes::MusclePropertiesDetailed ) ) {
			const auto& name = GetName();
			if ( !flags( StoreDataTypes::State ) ) // activation is also part of state
				frame[name + ".activation"] = GetActivation();

			// basic muscle properties
			frame[name + ".fiber_length_norm"] = GetNormalizedFiberLength();
			frame[name + ".fiber_velocity_norm"] = GetNormalizedFiberVelocity();
			frame[name + ".tendon_length_norm"] = GetNormalizedTendonLength() - 1;
			frame[name + ".mtu_force_norm"] = GetNormalizedForce();

			// detailed muscle properties
			if ( flags( StoreDataTypes::MusclePropertiesDetailed ) ) {
				// tendon / mtu properties
				frame[name + ".tendon_length"] = GetTendonLength();
				frame[name + ".mtu_length"] = GetLength();
				frame[name + ".mtu_velocity"] = GetVelocity();
				frame[name + ".mtu_force"] = GetForce();
				frame[name + ".mtu_power"] = GetForce() * GetVelocity();

				// fiber properties
				frame[name + ".cos_pennation_angle"] = GetCosPennationAngle();
				frame[name + ".force_length_multiplier"] = GetActiveForceLengthMultipler();
				frame[name + ".force_velocity_multiplier"] = GetForceVelocityMultipler();
				frame[name + ".passive_fiber_force_norm"] = GetPassiveFiberForce() / GetMaxIsometricForce();
			}
		}

		if ( flags( StoreDataTypes::MuscleDofMomentPower ) ) {
			for ( auto& d : GetDofs() ) {
				auto name = GetName() + "." + d->GetName();
				auto ma = GetMomentArm( *d );
				auto mom = GetForce() * ma;
				frame[name + ".moment_arm"] = ma;
				frame[name + ".moment"] = mom;
				frame[name + ".power"] = mom * d->GetVel();
			}
		}
	}

	PropNode Muscle::GetInfo() const
	{
		PropNode pn;
		pn["name"] = GetName();
		pn["origin"] = GetOriginBody().GetName();
		pn["insertion"] = GetInsertionBody().GetName();
		pn["max_isometric_force"] = GetMaxIsometricForce();
		pn["optimal_fiber_length"] = GetOptimalFiberLength();
		pn["tendon_slack_length"] = GetTendonSlackLength();
		pn["pennation_angle_at_optimal"] = GetPennationAngleAtOptimal();
		pn["max_contraction_velocity"] = GetMaxContractionVelocity();
		if ( auto v = GetTendonStrainAtOneNormForce(); v != 0 )
			pn["tendon_strain_at_one_force_norm"] = v;
		if ( auto v = GetPassiveFiberStrainAtOneNormForce(); v != 0 )
			pn["passive_fiber_strain_at_one_force_norm"] = v;
		pn.add_child( "path", GetPathInfo( GetLocalMusclePath() ) );

		return pn;
	}

	void Muscle::InitBodyJointDofs( const Body* b )
	{
		if ( auto* j = b->GetJoint() )
		{
			m_Joints.push_back( j );
			xo::append( m_Dofs, j->GetDofs() );
		}
	}

	void Muscle::InitJointsDofs()
	{
		SCONE_ASSERT( m_Joints.empty() && m_Dofs.empty() );
		const Body* const orgBody = &GetOriginBody();
		const Body* const insBody = &GetInsertionBody();
		const Body* const rootBody = GetModel().HasRootBody() ? &GetModel().GetRootBody() : nullptr;
		const Body* b = nullptr;

		// add joints, traversing from insertion to origin
		for ( b = insBody; b && b != orgBody && b != rootBody; b = b->GetParentBody() )
			InitBodyJointDofs( b );

		if ( b == rootBody )
		{
			// In this case, the muscle crosses the root body and
			// we need to add joints from insertion to root as well.
			for ( b = orgBody; b && b != rootBody; b = b->GetParentBody() )
				InitBodyJointDofs( b );
		}
	}
}
