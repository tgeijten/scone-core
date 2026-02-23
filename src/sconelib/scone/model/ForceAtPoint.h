#pragma once

#include "scone/core/Vec3.h"

namespace scone
{
	struct ForceAtPoint
	{
		Vec3 force;
		Vec3 point;

		Vec3 moment() const { return xo::cross_product( point, force ); }
	};

	inline ForceAtPoint& operator+=( ForceAtPoint& fv1, const ForceAtPoint& fv2 ) {
		auto l1 = xo::length( fv1.force );
		auto l2 = xo::length( fv2.force );
		fv1.force += fv2.force;
		auto total_l = l1 + l2;
		if ( total_l > Real( 0 ) )
			fv1.point = ( l1 * fv1.point + l2 * fv2.point ) / ( total_l );
		return fv1;
	}

	inline ForceAtPoint operator+( ForceAtPoint fv1, const ForceAtPoint& fv2 ) {
		return fv1 += fv2;
	}

	inline bool operator==( const ForceAtPoint& fv1, const ForceAtPoint& fv2 ) {
		return fv1.force == fv2.force && fv1.point == fv2.point;
	}
}
