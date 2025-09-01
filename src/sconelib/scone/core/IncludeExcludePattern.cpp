#include "IncludeExcludePattern.h"

#include "xo/geometry/mirror_tools.h"

namespace scone
{
	IncludeExcludePattern::IncludeExcludePattern( const PropNode& pn ) :
		INIT_MEMBER( pn, include, "*" ),
		INIT_MEMBER( pn, exclude, "" )
	{}

	IncludeExcludePattern::IncludeExcludePattern( const String& include_str, const String& exclude_str ) :
		include( include_str ),
		exclude( exclude_str )
	{}

	bool IncludeExcludePattern::match( const string& str ) const
	{
		return include.match( str ) && !exclude.match( str );
	}

	void IncludeExcludePattern::mirror_patterns()
	{
		xo::mirror( include );
		xo::mirror( exclude );
	}
}
