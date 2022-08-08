/*
** State.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "State.h"

#include <algorithm>
#include "scone/core/Exception.h"
#include "xo/string/string_tools.h"

namespace scone
{
	State::State( std::vector<String>&& names ) :
		names_( std::move( names ) )
	{
		values_.resize( names_.size() );
	}

	index_t State::AddVariable( const String& name, Real value )
	{
		names_.push_back( name );
		values_.push_back( value );
		return names_.size() - 1;
	}

	void State::SetValue( const String& name, Real value )
	{
		auto idx = FindIndex( name );
		SCONE_ERROR_IF( idx == NoIndex, "Could not find state: " + name );
		values_[ idx ] = value;
	}

	void State::TrySetValue( const String& name, Real value )
	{
		if ( auto idx = FindIndex( name ); idx != NoIndex )
			values_[ idx ] = value;
	}

	void State::SetValues( const std::vector<Real>& v )
	{
		SCONE_ASSERT( values_.size() <= v.size() );
		xo::copy( v.begin(), v.begin() + values_.size(), values_.begin() );
	}

	index_t State::FindIndex( const String& name ) const
	{
		auto it = std::find( names_.begin(), names_.end(), name );
		if ( it != names_.end() )
			return it - names_.begin();
		else return NoIndex;
	}

	index_t State::FindIndexByPattern( const String& pattern, index_t idx ) const
	{
		for ( ; idx < names_.size(); ++idx )
			if ( xo::pattern_match( names_[ idx ], pattern ) )
				return idx;
		return NoIndex;
	}
}
