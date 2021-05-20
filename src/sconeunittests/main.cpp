/*
** main.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "scone/sconelib_config.h"
#include "xo/serialization/serialize.h"
#include "xo/system/log_sink.h"
#include "xo/system/test_case.h"
#include "xo/serialization/prop_node_serializer_zml.h"
#include "scenario_test.h"
#include "xo/utility/arg_parser.h"

int main( int argc, const char* argv[] )
{
	xo::log::console_sink sink( xo::log::level::info );
	try
	{
		scone::Initialize();

		auto args = xo::arg_parser( argc, argv );

		sink.set_log_level( xo::log::level( args.get<int>( "l", int( xo::log::level::info ) ) ) );

		if ( !args.has_flag( "skip-tutorials" ) && !args.has_flag( "skip-opensim" ) )
			scone::add_scenario_tests( "scenarios/Tutorials" );
#if SCONE_OPENSIM_3_ENABLED
		if ( !args.has_flag( "skip-opensim3" ) && !args.has_flag( "skip-opensim" ) ) {
			scone::add_scenario_tests( "scenarios/UnitTests/OpenSim3" );
			scone::add_scenario_tests( "scenarios/Examples", "*OpenSim3*.scone" );
		}
#endif
#if SCONE_OPENSIM_4_ENABLED
		if ( !args.has_flag( "skip-opensim4" ) && !args.has_flag( "skip-opensim" ) ) {
			//scone::add_scenario_tests( "scenarios/UnitTests/OpenSim4" );
			scone::add_scenario_tests( "scenarios/Examples", "*OpenSim4*.scone" );
		}
#endif
#if SCONE_HYFYDY_ENABLED
		if ( !args.has_flag( "skip-hyfydy" ) ) {
			scone::add_scenario_tests( "scenarios/UnitTests/Hyfydy", "*.par" );
			scone::add_scenario_tests( "scenarios/Examples", "*Hyfydy*.scone" );
		}
#endif

		return xo::test::run_tests_async();
	}
	catch ( std::exception& e )
	{
		xo::log::critical( e.what() );
		return -1;
	}
}
