/*
** Angle.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "platform.h"
#include "types.h"
#include "xo/geometry/angle.h"
#include "xo/numerical/bounds.h"
#include "xo/geometry/vec3_type.h"

namespace scone
{
	// import Radian and Degree from xo
	using Degree = xo::degree_< Real >;
	using Radian = xo::radian_< Real >;
	using BoundsRad = xo::bounds< Radian >;
	using BoundsDeg = xo::bounds< Degree >;
	using Bounds3Rad = xo::vec3_< BoundsRad >;
	using Bounds3Deg = xo::vec3_< BoundsDeg >;
}

