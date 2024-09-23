/*
** Spring.h
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
	class Body;

	class SCONE_API Spring : HasData
	{
	public:
		Spring() = default;
		virtual ~Spring() = default;

		virtual const Body& GetParentBody() const = 0;
		virtual const Body& GetChildBody() const = 0;
		virtual Vec3 GetPosInParent() const = 0;
		virtual Vec3 GetPosInChild() const = 0;
		virtual Real GetRestLength() const = 0;
		virtual Real GetStiffness() const = 0;
		virtual Real GetDamping() const = 0;

		virtual Vec3 GetParentPos() const;
		virtual Vec3 GetChildPos() const;
		virtual bool IsActive() const;

		virtual void SetParent( const Body& b, const Vec3& pos ) = 0;
		virtual void SetChild( const Body& b, const Vec3& pos ) = 0;
		virtual void SetRestLength( Real l ) = 0;
		virtual void SetStiffness( Real kp ) = 0;
		virtual void SetDamping( Real kd ) = 0;

		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;
		virtual void SetStateFromData( const Storage<Real>::Frame& frame );
	};
}
