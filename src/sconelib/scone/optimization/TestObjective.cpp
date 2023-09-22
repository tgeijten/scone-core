/*
** TestObjective.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "TestObjective.h"

#include "xo/string/string_tools.h"
#include "scone/core/string_tools.h"
#include "xo/numerical/math.h"
#include "spot/test_objectives.h"

namespace scone
{
	auto get_function_ptr( TestObjectiveFunction fun )
	{
		switch ( fun )
		{
		case TestObjectiveFunction::Sphere: return &spot::sphere;
		case TestObjectiveFunction::Ellipsoid: return &spot::sphere;
		case TestObjectiveFunction::Rosenbrock: return &spot::rosenbrock;
		case TestObjectiveFunction::Schwefel:return &spot::schwefel;
		case TestObjectiveFunction::Rastrigin: return &spot::rastrigin;
		default: xo_error( "Unsupported test function" );
		}
	}

	xo::boundsd get_range( TestObjectiveFunction fun )
	{
		switch ( fun )
		{
		case TestObjectiveFunction::Sphere: return { -1e9, 1e9 };
		case TestObjectiveFunction::Ellipsoid: return { -1e9, 1e9 };
		case TestObjectiveFunction::Rosenbrock: return { -5, 10 };
		case TestObjectiveFunction::Schwefel:return { -500, 500 };
		case TestObjectiveFunction::Rastrigin: return { -5.12, 5.12 };
		default: xo_error( "Unsupported test function" );
		}
	}

	TestObjective::TestObjective( const PropNode& pn, const path& find_file_folder ) :
		Objective( pn, find_file_folder ),
		INIT_MEMBER( pn, function_, TestObjectiveFunction::Schwefel ),
		INIT_MEMBER( pn, dim_, 10 ),
		INIT_MEMBER( pn, range_, get_range( function_ ) ),
		INIT_MEMBER( pn, mean_, range_.center() ),
		INIT_MEMBER( pn, std_, range_.length() / 5 ),
		fun_( get_function_ptr( function_ ) )
	{
		auto range = get_range( function_ );
		for ( index_t i = 0; i < dim_; ++i )
			info_.add( ParInfo( stringf( "P%d", i ), mean_, std_, range_.lower, range_.upper ) );
	}

	fitness_t TestObjective::evaluate( const SearchPoint& point ) const
	{
		return fun_( point.values() );
	}

	String TestObjective::GetClassSignature() const
	{
		return to_str( function_ ) + to_str( dim_ );
	}
}
