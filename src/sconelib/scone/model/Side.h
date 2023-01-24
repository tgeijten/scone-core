/*
** Side.h
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/core/platform.h"
#include "scone/core/memory_tools.h"
#include "scone/core/Exception.h"
#include <vector>

#include <algorithm>

#include "xo/string/string_tools.h"
#include "xo/string/string_cast.h"
#include "scone/core/types.h"
#include "scone/core/Vec3.h"

namespace scone
{
	enum class Side {
		Left = -1,
		None = 0,
		Right = 1,
		Opposite = 999
	};

	inline Side GetOppositeSide( Side s ) {
		switch ( s )
		{
		case Side::Left: return Side::Right;
		case Side::None: return Side::None;
		case Side::Right: return Side::Left;
		default: SCONE_THROW( "Cannot determine opposite side" );
		}
	}

	inline Side GetSideFromName( const String& str )
	{
		if ( str.length() >= 2 )
		{
			String substr = xo::to_lower( str.substr( str.size() - 2 ) );
			if ( substr == "_r" ) return Side::Right;
			else if ( substr == "_l" ) return Side::Left;
			else if ( substr == "_o" ) return Side::Opposite;
		}

		return Side::None;
	}

	inline String GetNameNoSide( const String& str )
	{
		if ( GetSideFromName( str ) != Side::None )
			return str.substr( 0, str.length() - 2 );
		else return str;
	}

	inline String GetSideName( const Side& side )
	{
		if ( side == Side::Left ) return "_l";
		else if ( side == Side::Right ) return "_r";
		else if ( side == Side::Opposite ) return "_o";
		else return "";
	}

	inline String GetFullSideName( const Side& side )
	{
		if ( side == Side::Left ) return "Left";
		else if ( side == Side::Right ) return "Right";
		else if ( side == Side::Opposite ) return "Opposite";
		else return "Side::None";
	}

	inline String GetSidedName( const String& str, const Side& side )
	{
		return GetNameNoSide( str ) + GetSideName( side );
	}

	inline String GetSidedNameIfUnsided( const String& str, const Side& side )
	{
		auto current_side = GetSideFromName( str );
		if ( current_side == Side::None ) return GetSidedName( str, side );
		else if ( current_side == Side::Opposite ) return GetSidedName( str, GetOppositeSide( side ) );
		else return str;
	}

	inline String GetMirroredName( const String& str )
	{
		return GetNameNoSide( str ) + GetSideName( GetOppositeSide( GetSideFromName( str ) ) );
	}

	inline bool IsMirrored( const String& str1, const String& str2 ) { return str1 == GetMirroredName( str2 ); }

	// get axis that is mirrored in the XY plane for Side::Left, used in e.g. BodyOrientationSensor
	inline Vec3 GetSidedAxis( Vec3 axis, Side side ) {
		if ( side == Side::Left ) { axis.x = -axis.x; axis.y = -axis.y; }
		return axis;
	}

	// get direction vector that is mirrored in the XY plane for Side::Left, used in e.g. ComBosSensor
	inline Vec3 GetSidedDirection( Vec3 dir, Side side ) {
		if ( side == Side::Left ) dir.z = -dir.z;
		return dir;
	}

	inline Side GetSideOr( Side s1, Side s2 ) {
		return s1 == Side::None ? s2 : s1;
	}
	inline Real GetSidedAxisScale( index_t axis, Side s ) {
		return axis != 2 && s == Side::Left ? -1.0 : 1.0;
	}

	template< typename T >
	const T& FindByNameTrySided( const std::vector< T >& cont, const String& name, const Side& side )
	{
		auto it = std::find_if( cont.begin(), cont.end(), [&]( const T& item ) { return item->GetName() == name; } );
		if ( it != cont.end() )
			return *it;

		auto sided_name = GetSidedNameIfUnsided( name, side );
		if ( name != sided_name ) {
			it = std::find_if( cont.begin(), cont.end(), [&]( const T& item ) { return item->GetName() == sided_name; } );
			if ( it != cont.end() )
				return *it;
		}

		SCONE_THROW( "Could not find " + xo::quoted( name ) + " or " + xo::quoted( sided_name ) );
	}

	inline Vec3 GetMirrored( Vec3 v ) { v.z = -v.z; return v; }
	inline bool IsMirrored( const Vec3& v1, const Vec3& v2 ) { return v1 == GetMirrored( v2 ); }
}

namespace xo
{
	inline bool from_str( const string& s, scone::Side& v )
	{
		if ( str_equals_any_of( s, { "left", "l" } ) )
			v = scone::Side::Left;
		else if ( str_equals_any_of( s, { "right", "r" } ) )
			v = scone::Side::Right;
		else if ( str_equals_any_of( s, { "both", "none" } ) )
			v = scone::Side::None;
		else if ( str_equals_any_of( s, { "opposite", "other" } ) )
			v = scone::Side::Opposite;
		else return false; // could not extract side
		return true;
	}
}
