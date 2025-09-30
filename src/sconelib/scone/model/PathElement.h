#pragma once

#include "Body.h"
#include "scone/core/Vec3.h"
#include "scone/core/Quat.h"

namespace scone
{
	struct PathElement
	{
		PathElement( const Body* b, const Vec3& p, const Quat& o = Quat::identity(), Real r = Real( 0 ) ) :
			body( b ), pos( p ), ori( o ), radius( r )
		{}

		const Body* body;
		Vec3 pos;
		Quat ori;
		Real radius;
	};
}

