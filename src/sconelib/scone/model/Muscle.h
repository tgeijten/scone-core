/*
** Muscle.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/core/platform.h"
#include "scone/core/types.h"
#include "scone/core/HasData.h"
#include "scone/core/Vec3.h"
#include "scone/core/Storage.h"

#include "Actuator.h"
#include "Side.h"

#include <vector>

namespace scone
{
	class SCONE_API Muscle : public Actuator
	{
	public:
		Muscle( const class Model& model );
		virtual ~Muscle();

		virtual Real GetMinInput() const override { return m_MinActivation; }
		virtual Real GetMaxInput() const override { return m_MaxActivation; }

		virtual double GetAdaptedInput() const override;

		virtual const Body& GetOriginBody() const = 0;
		virtual const Body& GetInsertionBody() const = 0;
		virtual const Model& GetModel() const = 0;

		virtual Real GetMomentArm( const Dof& dof ) const = 0;
		virtual Vec3 GetMomentArm3D( const Joint& joint ) const = 0;
		virtual Real GetNormalizedMomentArm( const Dof& dof ) const;
		virtual Real GetMoment( const Dof& dof ) const;

		virtual Real GetMaxIsometricForce() const = 0;
		virtual Real GetOptimalFiberLength() const = 0;
		virtual Real GetTendonSlackLength() const = 0;
		virtual Real GetPennationAngleAtOptimal() const = 0;

		virtual Real GetMass( Real specific_tension = 0.25e6, Real muscle_density = 1059.7 ) const;
		virtual Real GetPCSA( Real specific_tension = 0.25e6 ) const;

		virtual Real GetForce() const = 0;
		virtual Real GetNormalizedForce() const = 0;

		virtual Real GetLength() const = 0;
		virtual Real GetVelocity() const = 0;
		virtual Real GetNormalizedSpindleRate() const;

		virtual Real GetFiberForce() const = 0;
		virtual Real GetActiveFiberForce() const = 0;
		virtual Real GetPassiveFiberForce() const = 0;

		virtual Real GetFiberLength() const = 0;
		virtual Real GetNormalizedFiberLength() const = 0;

		virtual Real GetCosPennationAngle() const = 0;

		virtual Real GetFiberVelocity() const = 0;
		virtual Real GetNormalizedFiberVelocity() const = 0;

		virtual Real GetTendonLength() const = 0;
		virtual Real GetNormalizedTendonLength() const = 0;

		virtual Real GetActiveForceLengthMultipler() const = 0;
		virtual Real GetForceVelocityMultipler() const = 0;
		virtual Real GetMaxContractionVelocity() const = 0;

		virtual std::vector< Vec3 > GetMusclePath() const = 0;
		virtual std::vector< std::pair< Body*, Vec3 > > GetLocalMusclePath() const { SCONE_THROW_NOT_IMPLEMENTED; }

		virtual Real GetActivation() const = 0;
		virtual Real GetExcitation() const = 0;
		virtual void SetExcitation( Real u ) = 0;
		virtual void InitializeActivation( Real u ) = 0;
		virtual void SetMinActivation( Real min_act ) { m_MinActivation = min_act; }
		virtual void SetMaxActivation( Real max_act ) { m_MaxActivation = max_act; }

		virtual Side GetSide() const;

		virtual bool HasMomentArm( const Dof& dof ) const;
		virtual const std::vector< const Joint* >& GetJoints() const;
		virtual const std::vector< const Dof* >& GetDofs() const;
		virtual bool IsAntagonist( const Muscle& other ) const;
		virtual bool IsAgonist( const Muscle& other ) const;
		virtual bool HasSharedDofs( const Muscle& other ) const;
		virtual bool HasSharedBodies( const Muscle& other ) const;
		virtual bool HasSharedJoints( const Muscle& other ) const;

		virtual void StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const override;
		virtual PropNode GetInfo() const override;

		Real GetClampedInput() const { return xo::clamped( m_ActuatorInput, m_MinActivation, m_MaxActivation ); }
		Real GetSoftLimitInput( Real lb, Real ub ) const { return xo::smooth_clamped( m_ActuatorInput, m_MinActivation, m_MaxActivation, lb, ub ); }

		virtual Real SetMaxIsometricForce( Real force ) { SCONE_THROW_NOT_IMPLEMENTED; }
		virtual Real SetOptimalFiberLength( Real length ) { SCONE_THROW_NOT_IMPLEMENTED; }
		virtual Real SetTendonSlackLength( Real length ) { SCONE_THROW_NOT_IMPLEMENTED; }
		virtual Real SetPennationAngleAtOptimal( Real angle ) { SCONE_THROW_NOT_IMPLEMENTED; }

	protected:
		void InitBodyJointDofs( const Body* b );
		void InitJointsDofs();
		mutable std::vector< const Joint* > m_Joints;
		mutable std::vector< const Dof* > m_Dofs;
		Real m_MinActivation;
		Real m_MaxActivation;
	};
}
