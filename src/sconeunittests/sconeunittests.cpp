/*
** main.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
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
#include "scone/core/system_tools.h"

int main( int argc, const char* argv[] )
{
	xo::log::console_sink sink( xo::log::level::info );
	try
	{
		auto args = xo::arg_parser( argc, argv );
		sink.set_log_level( xo::log::level( args.get<int>( "l", int( xo::log::level::info ) ) ) );

		xo::log::info( "SCONE version ", scone::GetSconeVersion() );
		scone::Initialize();
		auto& install_dir = scone::GetInstallFolder();

		if ( !args.has_flag( "skip-scenarios" ) ) {
#if SCONE_OPENSIM_3_ENABLED
			if ( !args.has_flag( "skip-opensim3" ) && !args.has_flag( "skip-opensim" ) ) {
				scone::add_scenario_tests( install_dir, "scenarios/UnitTests/OpenSim3" );
				scone::add_scenario_tests( install_dir, "scenarios/Examples", "*OpenSim3*.scone" );
				scone::add_scenario_tests( install_dir, "scenarios/Examples2", "*OpenSim3*.scone" );
				scone::add_scenario_tests( install_dir, "scenarios/Tutorials2", "*OpenSim.scone" );
				if ( !args.has_flag( "skip-tutorials" ) ) {
					scone::add_scenario_tests( install_dir, "scenarios/Tutorials" );
				}
			}
#endif
#if SCONE_OPENSIM_4_ENABLED
			if ( !args.has_flag( "skip-opensim4" ) && !args.has_flag( "skip-opensim" ) ) {
				//scone::add_scenario_tests( install_dir,  "scenarios/UnitTests/OpenSim4" );
				scone::add_scenario_tests( install_dir, "scenarios/Examples", "*OpenSim4*.scone" );
				scone::add_scenario_tests( install_dir, "scenarios/Examples2", "*OpenSim4*.scone" );
			}
#endif
#if SCONE_HYFYDY_ENABLED
			if ( !args.has_flag( "skip-hyfydy" ) ) {
				scone::add_scenario_tests( install_dir, "scenarios/UnitTests/HyfydyResults", "*.par", "", 1 );
				scone::add_scenario_tests( install_dir, "scenarios/UnitTests/Hyfydy", "*.scone", "", 1 );
				scone::add_scenario_tests( install_dir, "scenarios/Examples", "*Hyfydy.scone" );
				scone::add_scenario_tests( install_dir, "scenarios/Examples2", "*Hyfydy.scone" );
				scone::add_scenario_tests( install_dir, "scenarios/Tutorials2", "*Hyfydy.scone" );
				scone::add_scenario_tests( install_dir, "scenarios/Benchmarks", "*.par", "data", 1 );
				scone::add_scenario_tests( scone::GetDataFolder(), "Benchmarks", "bench*.scone" );
#	if SCONE_EXPERIMENTAL_FEATURES_ENABLED
				scone::add_scenario_tests( install_dir, "scenarios/UnitTests/HyfydyExperimental", "*.par", "data", 1 );
				if ( args.has_flag( "add-blueprints" ) )
					scone::add_scenario_tests( install_dir, "scenarios/UnitTests/Blueprints", "*.par", "data", 1 );
#	endif
			}
#endif
		}

		if ( args.has_flag( "sync" ) )
			return xo::test::run_tests();
		else
			return xo::test::run_tests_async();
	}
	catch ( std::exception& e )
	{
		xo::log::critical( e.what() );
		return -1;
	}
}
