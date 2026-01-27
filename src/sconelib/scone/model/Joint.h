/*
** Joint.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/core/HasName.h"
#include "scone/core/HasData.h"
#include "scone/core/PropNode.h"
#include "scone/core/Vec3.h"
#include "scone/core/Quat.h"

namespace scone
{
	class Dof;
	class Body;
	class Joint;

	class SCONE_API Joint : public HasName, HasData
	{
	public:
		Joint( Body& body, Body& parent_body );
		virtual ~Joint();

		virtual Vec3 GetPos() const = 0;
		virtual Vec3 GetPosInParent() const = 0;
		virtual Vec3 GetPosInChild() const = 0;
		virtual Quat GetOriInParent() const = 0;
		virtual Quat GetOriInChild() const = 0;

		virtual Vec3 GetReactionForce() const = 0;
		virtual Vec3 GetLimitTorque() const;
		virtual Real GetLimitPower() const;
		virtual Vec3 GetMuscleMoment() const;
		virtual Real GetLoad() const;

		const Body& GetBody() const { return m_Body; }
		const Body& GetParentBody() const { return m_ParentBody; }

		virtual bool HasMotor() const { return false; }
		virtual Real GetMotorMaxTorque() const { return 0.0; }
		virtual void SetMotorTargetOri( const Quat& o ) { SCONE_THROW_NOT_IMPLEMENTED; }
		virtual void SetMotorTargetVel( const Vec3& v ) { SCONE_THROW_NOT_IMPLEMENTED; }
		virtual void AddMotorTorque( const Vec3& v ) { SCONE_THROW_NOT_IMPLEMENTED; }
		virtual Vec3 GetMotorTorque() const { return Vec3::zero(); }
		virtual void SetMotorStiffness( Real kp ) { SCONE_THROW_NOT_IMPLEMENTED; }
		virtual void SetMotorDamping( Real kd ) { SCONE_THROW_NOT_IMPLEMENTED; }

		const std::vector< Dof* >& GetDofs() const;
		void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;
		virtual PropNode GetInfo() const;

	protected:
		Body& m_Body;
		Body& m_ParentBody;
		mutable std::vector< Dof* > m_Dofs;
	};
}
