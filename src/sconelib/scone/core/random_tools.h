#pragma once

#include "PropNode.h"
#include "scone/optimization/Params.h"

namespace scone
{
	// get a random_seed from pn
	// if the seed is zero, a deterministic seed will be derived from par
	// this is an awesome way to get consistent randomized seeds
	inline unsigned int GetRandomSeed( const PropNode& pn, const Params& par ) {
		auto seed = pn.get<unsigned int>( "random_seed", 123 );
		if ( seed == 0 ) {
			if ( auto* sp = dynamic_cast<const SearchPoint*>( &par ) ) {
				xo::uint64 i = 0;
				for ( auto& v : sp->values() )
					i ^= *reinterpret_cast<const xo::uint64*>( &v );
				seed = static_cast<unsigned int>( i >> 32 ^ i );
			}
		}
		return seed;
	}
}
