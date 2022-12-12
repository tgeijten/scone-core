/*
** model_tools.h
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include <vector>
#include "scone/core/platform.h"
#include "scone/core/types.h"
#include "scone/core/Vec3.h"
#include "scone/core/math.h"
#include "scone/core/PropNode.h"

namespace scone
{
	SCONE_API Vec3 GetGroundCop( const Vec3& force, const Vec3& moment, Real min_force = REAL_WIDE_EPSILON );
	SCONE_API Vec3 GetPlaneCop( const Vec3& normal, const Vec3& location, const Vec3& force, const Vec3& moment, Real min_force = REAL_WIDE_EPSILON );

	SCONE_API PropNode MakePropNode( const std::vector< UserInputUP >& user_inputs );
	SCONE_API size_t SetUserInputsFromPropNode( const PropNode& pn, const std::vector<UserInputUP>& user_inputs );

	SCONE_API bool IsRealJoint( const Joint& j );
	SCONE_API string GetDofSourceName( const Dof& dof );
}
