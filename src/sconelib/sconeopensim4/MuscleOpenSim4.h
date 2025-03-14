/*
** MuscleOpenSim4.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "platform.h"
#include "scone/model/Muscle.h"
#include "xo/container/circular_deque.h"

#include <vector>
#include "xo/container/flat_map.h"

#define ENABLE_MOMENT_ARM_CACHE 0

namespace OpenSim
{
	class Muscle;
}

namespace scone
{
	class ModelOpenSim4;

	class SCONE_OPENSIM_4_API MuscleOpenSim4 : public Muscle
	{
	public:
		MuscleOpenSim4( ModelOpenSim4& model, OpenSim::Muscle& mus );
		virtual ~MuscleOpenSim4();

		// access to bodies
		virtual const Body& GetOriginBody() const override;
		virtual const Body& GetInsertionBody() const override;
		virtual const Model& GetModel() const override;

		// muscle parameters
		virtual Real GetMaxIsometricForce() const override;
		virtual Real GetOptimalFiberLength() const override;
		virtual Real GetTendonSlackLength() const override;
		virtual Real GetPennationAngleAtOptimal() const override;

		// current force / length / velocity
		virtual Real GetForce() const override;
		virtual Real GetNormalizedForce() const override;

		virtual Real GetLength() const override;
		virtual Real GetVelocity() const override;

		virtual Real GetFiberForce() const override;
		virtual Real GetActiveFiberForce() const override;
		virtual Real GetPassiveFiberForce() const override;

		virtual Real GetFiberLength() const override;
		virtual Real GetNormalizedFiberLength() const override;

		virtual Real GetCosPennationAngle() const override;

		virtual Real GetFiberVelocity() const override;
		virtual Real GetNormalizedFiberVelocity() const override;

		virtual Real GetTendonLength() const override;
		virtual Real GetNormalizedTendonLength() const override;

		virtual Real GetActiveForceLengthMultipler() const override;
		virtual Real GetForceVelocityMultipler() const override;
		virtual Real GetMaxContractionVelocity() const override;

		virtual Real GetActivation() const override;
		virtual Real GetExcitation() const override;

		virtual std::vector< Vec3 > GetMusclePath() const override;
		virtual std::vector< std::pair< Body*, Vec3 > > GetLocalMusclePath() const override;

		virtual void SetExcitation( Real u ) override;
		virtual void InitializeActivation( Real a ) override;

		OpenSim::Muscle& GetOsMuscle() { return m_osMus; }

		virtual const String& GetName() const override;
		virtual Real GetMomentArm( const Dof& dof ) const override;
		virtual Vec3 GetMomentArm3D( const Joint& joint ) const override { return Vec3::zero(); } // not supported

		void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;

	private:
		ModelOpenSim4& m_Model;
		OpenSim::Muscle& m_osMus;
#if ENABLE_MOMENT_ARM_CACHE
		mutable TimeInSeconds m_MomentArmCacheTimeStamp = -1.0;
		mutable xo::flat_map< const Dof*, Real > m_MomentArmCache;
#endif
	};
}
