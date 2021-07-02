#pragma once

#include "scone/core/Vec3.h"
#include "scone/core/Quat.h"
#include "scone/core/types.h"
#include "xo/filesystem/path.h"
#include "xo/shape/shape.h"

namespace scone
{
	struct DisplayGeometry
	{
		DisplayGeometry( const xo::path& file, const Vec3& p, const Quat& q = Quat(), const Vec3& s = Vec3::diagonal( 1 ) ) :
			filename_( file ),
			shape_(),
			pos_( p ),
			ori_( q ),
			scale_( s )
		{}
		DisplayGeometry( const xo::shape& sh, const Vec3& p, const Quat& q = Quat() ) :
			filename_(),
			shape_( sh ),
			pos_( p ),
			ori_( q ),
			scale_( Vec3::diagonal( 1 ) )
		{}

		xo::path filename_;
		xo::shape shape_;
		Vec3 pos_;
		Quat ori_;
		Vec3 scale_;
	};
}
