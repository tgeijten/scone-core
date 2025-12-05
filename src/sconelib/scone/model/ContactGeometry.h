/*
** ContactGeometry.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see https://scone.software.
*/

#pragma once

#include "scone/core/platform.h"
#include "Body.h"
#include "scone/core/Vec3.h"
#include "scone/core/Quat.h"
#include "xo/shape/shape.h"
#include "scone/core/HasName.h"

namespace scone
{
	class SCONE_API ContactGeometry : public HasName
	{
	public:
		ContactGeometry( const String& name, const Body& b, const xo::shape& s, const Vec3& p, const Quat& q, const String& material );
		ContactGeometry( const String& name, const Body& b, const xo::path& f, const Vec3& p, const Quat& q, const String& material );

		virtual const String& GetName() const override { return m_Name; }

		const Body& GetBody() const { return m_Body; }
		const xo::shape& GetShape() const { return m_Shape; }
		const xo::path& GetFileName() const { return m_FileName; }
		bool HasFileName() const { return !m_FileName.empty(); }
		const Vec3& GetPos() const { return m_Pos; }
		const Quat& GetOri() const { return m_Ori; }
		PropNode GetInfo() const;

	private:
		const String& m_Name;
		const Body& m_Body;
		xo::shape m_Shape;
		Vec3 m_Pos;
		Quat m_Ori;
		xo::path m_FileName;
		const String& m_Material;
	};
}
