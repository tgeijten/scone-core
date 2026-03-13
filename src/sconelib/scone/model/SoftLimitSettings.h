#pragma once

#include "scone/core/types.h"
#include "scone/core/PropNode.h"
#include "scone/optimization/Params.h"

namespace scone
{
	struct SoftLimitSettings {
		SoftLimitSettings( const PropNode& pn, Params& par ) :
			INIT_PAR_MEMBER( pn, par, lower, 0.0 ),
			INIT_PAR_MEMBER( pn, par, upper, 1.0 ),
			INIT_MEMBER( pn, type, lower > 0.0 && upper < 1.0 ? 2 : 0 )
		{
			if ( pn.size() == 2 && pn.is_array() ) {

			}

		}

		Real lower;
		Real upper;
		enum Type { Clamp, SoftC1, SoftC2 } type;
	};
}
