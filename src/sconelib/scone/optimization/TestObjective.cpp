/*
** TestObjective.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "TestObjective.h"

#include "xo/string/string_tools.h"
#include "scone/core/string_tools.h"
#include "xo/numerical/math.h"

namespace scone
{
	double sphere( const spot::par_vec& v )
	{
		double sum = 0.0;
		for ( unsigned int i = 0; i < v.size(); ++i )
			sum += xo::squared( v[ i ] );
		return sum;
	}

	double rosenbrock( const spot::par_vec& v )
	{
		double sum = 0.0;
		for ( unsigned int i = 0; i < v.size() - 1; i++ )
			sum += 100 * xo::squared( v[ i + 1 ] - xo::squared( v[ i ] ) ) + xo::squared( 1. - v[ i ] );
		return sum;
	}

	double schwefel( const spot::par_vec& v )
	{
		double sum = 0.0;
		for ( index_t i = 0; i < v.size(); ++i )
			sum += v[ i ] * sin( sqrt( fabs( v[ i ] ) ) );
		return 418.9829 * v.size() - sum;
	}

	double rastrigin( const spot::par_vec& v )
	{
		double sum = 10.0 * v.size();
		for ( index_t i = 0; i < v.size(); ++i )
			sum += xo::squared( v[ i ] ) - 10.0 * cos( 2 * xo::constantsd::pi() * v[ i ] );
		return sum;
	}

	auto get_function_ptr( TestObjectiveFunction fun )
	{
		switch ( fun )
		{
		case TestObjectiveFunction::Sphere: return &sphere;
		case TestObjectiveFunction::Rosenbrock: return &rosenbrock;
		case TestObjectiveFunction::Schwefel:return &schwefel;
		case TestObjectiveFunction::Rastrigin: return &rastrigin;
		default: xo_error( "Unsupported test function" );
		}
	}

	xo::boundsd get_range( TestObjectiveFunction fun )
	{
		switch ( fun )
		{
		case TestObjectiveFunction::Sphere: return { -1e9, 1e9 };
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
		INIT_MEMBER( pn, std_, range_.range() / 5 ),
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
