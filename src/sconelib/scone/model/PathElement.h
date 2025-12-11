#pragma once

#include "Body.h"
#include "scone/core/Vec3.h"
#include "scone/core/Quat.h"

namespace scone
{
	struct PathElement
	{
		PathElement( const Body* b, const Vec3& p, const Vec3& d = Vec3::zero(), Real r = Real( 0 ) ) :
			body( b ), pos( p ), dir( d ), radius( r )
		{}

		const Body* body;
		Vec3 pos;
		Vec3 dir;
		Real radius;

		bool HasDir() const { return !dir.is_null(); }
		bool IsCylinder() const { return radius != 0; }
	};
}

