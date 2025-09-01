#pragma once

#include "scone/core/types.h"
#include "PropNode.h"
#include "xo/string/pattern_matcher.h"

namespace scone
{
	struct SCONE_API IncludeExcludePattern
	{
		IncludeExcludePattern() = default;
		IncludeExcludePattern( const PropNode& pn );
		IncludeExcludePattern( const String& include_str, const String& exclude_str );

		bool match( const string& str ) const;
		bool operator()( const string& str ) const { return match( str ); }
		void mirror_patterns();

	private:
		xo::pattern_matcher include;
		xo::pattern_matcher exclude;
	};
}

namespace xo
{
	inline bool from_prop_node( const prop_node& pn, scone::IncludeExcludePattern& v ) {
		v = scone::IncludeExcludePattern( pn );
		return true;
	}
}
