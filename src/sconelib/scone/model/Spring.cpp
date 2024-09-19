#include "Spring.h"

#include "Body.h"

namespace scone
{
	Vec3 Spring::GetParentPos() const
	{
		return GetParentBody().GetPosOfPointOnBody( GetPosInParent() );
	}

	Vec3 Spring::GetChildPos() const
	{
		return GetChildBody().GetPosOfPointOnBody( GetPosInChild() );
	}
}
