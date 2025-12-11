/*
** Ligament.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/core/platform.h"
#include "scone/core/types.h"
#include "scone/core/HasName.h"
#include "scone/core/HasData.h"
#include "scone/core/Vec3.h"
#include "scone/core/Storage.h"
#include "scone/core/PropNode.h"
#include "PathElement.h"
#include "Side.h"
#include <vector>

namespace scone
{
	class SCONE_API Ligament : public HasName, public HasData
	{
	public:
		Ligament( const class Model& model );
		virtual ~Ligament();

		virtual const Body& GetOriginBody() const = 0;
		virtual const Body& GetInsertionBody() const = 0;
		virtual const Model& GetModel() const = 0;

		virtual Real GetMomentArm( const Dof& dof ) const = 0;
		virtual Real GetNormalizedMomentArm( const Dof& dof ) const;
		virtual Real GetMoment( const Dof& dof ) const;

		virtual Real GetPcsaForce() const = 0;
		virtual Real GetRestingLength() const = 0;

		virtual Real GetForce() const = 0;
		virtual Real GetNormalizedForce() const = 0;

		virtual Real GetLength() const = 0;
		virtual Real GetNormalizedLength() const = 0;

		virtual Real GetVelocity() const = 0;
		virtual Real GetNormalizedVelocity() const = 0;

		virtual std::vector<Vec3> GetLigamentPath() const = 0;
		virtual std::vector<PathElement> GetLocalLigamentPath() const { SCONE_THROW_NOT_IMPLEMENTED; }

		virtual Side GetSide() const;

		virtual bool HasMomentArm( const Dof& dof ) const;
		virtual const std::vector<const Joint*>& GetJoints() const;
		virtual const std::vector<const Dof*>& GetDofs() const;

		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;
		virtual PropNode GetInfo() const;

	protected:
		void InitBodyJointDofs( const Body* b );
		void InitJointsDofs();
		mutable std::vector<const Joint*> m_Joints;
		mutable std::vector<const Dof*> m_Dofs;
		Real m_MinActivation;
		Real m_MaxActivation;
	};
}
