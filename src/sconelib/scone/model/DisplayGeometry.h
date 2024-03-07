#pragma once

#include "scone/core/Vec3.h"
#include "scone/core/Quat.h"
#include "scone/core/types.h"
#include "xo/filesystem/path.h"
#include "xo/shape/shape.h"
#include "xo/utility/color.h"

namespace scone
{
	struct DisplayGeometry
	{
		DisplayGeometry( const xo::path& file, const Vec3& p, const Quat& q = Quat(), const Vec3& s = Vec3::one(), const xo::color& c = xo::color::null() ) :
			filename_( file ),
			shape_(),
			pos_( p ),
			ori_( q ),
			scale_( s ),
			color_( c )
		{}
		DisplayGeometry( const xo::shape& sh, const Vec3& p, const Quat& q = Quat(), const Vec3& s = Vec3::one(), const xo::color& c = xo::color::null() ) :
			filename_(),
			shape_( sh ),
			pos_( p ),
			ori_( q ),
			scale_( s ),
			color_( c )
		{}

		xo::path filename_;
		xo::shape shape_;
		Vec3 pos_;
		Quat ori_;
		Vec3 scale_;
		xo::color color_;
	};
}
