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
#include "scone/core/version.h"

int main( int argc, const char* argv[] )
{
	xo::log::console_sink sink( xo::log::level::info );
	try
	{
		auto args = xo::arg_parser( argc, argv );
		sink.set_log_level( xo::log::level( args.get<int>( "l", int( xo::log::level::info ) ) ) );

		scone::Initialize();
		xo::log::info( "SCONE version ", scone::GetSconeVersion() );

		if ( !args.has_flag( "skip-scenarios" ) ) {
			if ( !args.has_flag( "skip-tutorials" ) && !args.has_flag( "skip-opensim" ) ) {
				scone::add_scenario_tests( "scenarios/Tutorials" );
			}
#if SCONE_OPENSIM_3_ENABLED
			if ( !args.has_flag( "skip-opensim3" ) && !args.has_flag( "skip-opensim" ) ) {
				scone::add_scenario_tests( "scenarios/UnitTests/OpenSim3" );
				scone::add_scenario_tests( "scenarios/Examples", "*OpenSim3*.scone" );
				scone::add_scenario_tests( "scenarios/Examples2", "*OpenSim3*.scone" );
				scone::add_scenario_tests( "scenarios/Tutorials2", "*OpenSim.scone" );
			}
#endif
#if SCONE_OPENSIM_4_ENABLED
			if ( !args.has_flag( "skip-opensim4" ) && !args.has_flag( "skip-opensim" ) ) {
				//scone::add_scenario_tests( "scenarios/UnitTests/OpenSim4" );
				scone::add_scenario_tests( "scenarios/Examples", "*OpenSim4*.scone" );
				scone::add_scenario_tests( "scenarios/Examples2", "*OpenSim4*.scone" );
			}
#endif
#if SCONE_HYFYDY_ENABLED
			if ( !args.has_flag( "skip-hyfydy" ) ) {
				scone::add_scenario_tests( "scenarios/UnitTests/Hyfydy", "*.par" );
				scone::add_scenario_tests( "scenarios/Examples", "*Hyfydy.scone" );
				scone::add_scenario_tests( "scenarios/Examples2", "*Hyfydy.scone" );
				scone::add_scenario_tests( "scenarios/Tutorials2", "*Hyfydy.scone" );

#if SCONE_EXPERIMENTAL_FEATURES_ENABLED
				scone::add_scenario_tests( "scenarios/UnitTests/HyfydyExperimental", "*.par" );
				if ( args.has_flag( "add-blueprints" ) )
					scone::add_scenario_tests( "scenarios/UnitTests/Blueprints", "*.par" );
#endif
			}
#endif
		}

		return xo::test::run_tests_async();
	}
	catch ( std::exception& e )
	{
		xo::log::critical( e.what() );
		return -1;
	}
}
