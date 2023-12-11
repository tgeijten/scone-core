#pragma once

#include "PropNode.h"
#include "xo/string/pattern_matcher.h"

namespace scone
{
	struct IncludeExcludePattern
	{
		IncludeExcludePattern( const PropNode& pn ) :
			INIT_MEMBER( pn, include, "*" ),
			INIT_MEMBER( pn, exclude, "" )
		{}

		bool match( const string& str ) const {
			return include.match( str ) && !exclude.match( str );
		}

		bool operator()( const string& str ) const { return match( str ); }

	private:
		xo::pattern_matcher include;
		xo::pattern_matcher exclude;
	};
}
