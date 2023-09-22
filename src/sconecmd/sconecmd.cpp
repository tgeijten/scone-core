/*
** sconecmd.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include <tclap/CmdLine.h>
#include "scone/core/Exception.h"
#include "scone/core/Factories.h"
#include "scone/core/Log.h"
#include "scone/core/version.h"
#include "scone/optimization/opt_tools.h"
#include "scone/sconelib_config.h"
#include "spot/optimizer_pool.h"
#include "xo/container/prop_node_tools.h"
#include "xo/serialization/prop_node_serializer_zml.h"
#include "xo/serialization/serialize.h"
#include "xo/system/log_sink.h"
#include "xo/system/system_tools.h"
#include "scone/core/Benchmark.h"
#include "xo/filesystem/filesystem.h"
#include "scone/core/system_tests.h"

using scone::PropNode;
using scone::String;
using scone::path;
using std::string;

// add custom arguments to prop_node
void handle_custom_arguments( PropNode& scenario_pn, const TCLAP::UnlabeledMultiArg< string >& propArg )
{
	for ( auto kvstring : propArg ) {
		auto kvp = xo::make_key_value_str( kvstring );
		scenario_pn.set_query( kvp.first, kvp.second, '.' );
	}
}

// load scenario and handle custom arguments
PropNode load_scenario( const path& scenario_file, const TCLAP::UnlabeledMultiArg< string >& propArg )
{
	PropNode scenario_pn = xo::load_file_with_include( scenario_file, "INCLUDE" );
	handle_custom_arguments( scenario_pn, propArg );
	return scenario_pn;
}

// main
int main( int argc, char* argv[] )
{
	xo::log::console_sink console_sink( xo::log::level::info );
	xo::log::info( "SCONE version ", scone::GetSconeVersion() );
	scone::Initialize();

	try
	{
		TCLAP::CmdLine cmd( "SCONE Command Line Utility", ' ', xo::to_str( scone::GetSconeVersion() ), true );
		TCLAP::ValueArg< String > optArg( "o", "optimize", "Optimize a scenario file", true, "", "*.scone" );
		TCLAP::ValueArg< String > parArg( "e", "evaluate", "Evaluate a result from an optimization", false, "", "*.par" );
		TCLAP::ValueArg< String > benchArg( "b", "benchmark", "Benchmark a scenario or parameter file", false, "", "*.scone" );
		TCLAP::ValueArg< int > bxArg( "x", "benchmarkx", "Number of benchmarks to perform", false, 8, ">0", cmd );
		TCLAP::ValueArg< String > outArg( "r", "result", "Output file for evaluation result", false, "", "Output file (*.sto)", cmd );
		TCLAP::ValueArg< int > logArg( "l", "log", "Set the log level", false, 1, "1-7", cmd );
		TCLAP::ValueArg< String > testArg( "", "test", "Perform test (internal use only)", false, "", "" );
		TCLAP::SwitchArg statusOutput( "s", "status", "Output full status updates", cmd, false );
		TCLAP::SwitchArg quietOutput( "q", "quiet", "Do not output simulation progress", cmd, false );
		TCLAP::UnlabeledMultiArg< string > propArg( "property", "Override specific scenario property, using <key>=<value>", false, "<key>=<value>", cmd, true );
#if SCONE_HYFYDY_ENABLED
		std::vector<string> hyfydyOptions{ "id", "key" };
		TCLAP::ValuesConstraint<string> allowedVals( hyfydyOptions );
		TCLAP::ValueArg< String > licenseArg( "", "hyfydy", "Hyfydy license key management", true, "", &allowedVals );
		auto xor_args = std::vector<TCLAP::Arg*>{ &optArg, &parArg , &benchArg, &licenseArg, &testArg };
#else
		auto xor_args = std::vector<TCLAP::Arg*>{ &optArg, &parArg , &benchArg };
#endif
		cmd.xorAdd( xor_args );
		cmd.parse( argc, argv );

		try
		{
			// set log level (optimization defaults to info, evaluation defaults to trace)
			if ( logArg.isSet() )
				console_sink.set_log_level( xo::log::level( logArg.getValue() ) );

			// do optimization or evaluation
			if ( optArg.isSet() )
			{
				path scenario_file = scone::FindScenario( optArg.getValue() );
				auto scenario_pn = load_scenario( scenario_file, propArg );
				if ( outArg.isSet() && scenario_pn.count_children() >= 1 )
					scenario_pn.front().second.set( "output_root", outArg.getValue() );
				scone::OptimizerUP o = scone::CreateOptimizer( scenario_pn, scenario_file.parent_path() );
				scone::LogUnusedProperties( scenario_pn );
				if ( statusOutput.getValue() )
					o->SetOutputMode( scone::Optimizer::status_console_output );
				else o->SetOutputMode( quietOutput.getValue() ? scone::Optimizer::no_output : scone::Optimizer::console_output );
				o->Run();
			}
			else if ( parArg.isSet() )
			{
				auto scenario_pn = scone::LoadScenario( parArg.getValue() );
				handle_custom_arguments( scenario_pn, propArg );
				auto out_path = path( outArg.isSet() ? outArg.getValue() : parArg.getValue() );
				scone::log::info( "Evaluating ", parArg.getValue() );
				auto results = scone::EvaluateScenario( scenario_pn, parArg.getValue(), out_path );
				scone::log::info( results );

				// store config file if arguments have changed
				if ( propArg.isSet() && outArg.isSet() )
					save_file( scenario_pn, out_path.replace_extension( "scone" ) );
			}
			else if ( benchArg.isSet() )
			{
				auto filename = xo::path( benchArg.getValue() );
				scone::BenchmarkOptions bopt;
				bopt.min_samples = bxArg.getValue();
				if ( xo::directory_exists( filename ) )
				{
					auto files = xo::find_files( filename, "*.par", true, 1 );
					for ( const auto& f : files )
					{
						auto scenario_pn = load_scenario( scone::FindScenario( f ), propArg );
						scone::BenchmarkScenario( scenario_pn, f, filename / "_benchmark_results", bopt );
					}
				}
				else
				{
					auto f = path( benchArg.getValue() );
					auto scenario_pn = load_scenario( scone::FindScenario( benchArg.getValue() ), propArg );
					scone::BenchmarkScenario( scenario_pn, f, f.parent_path() / "_benchmark_results", bopt );
				}
			}
			else if ( testArg.isSet() )
			{
				scone::perform_test( testArg.getValue() );
			}
#if SCONE_HYFYDY_ENABLED
			else if ( licenseArg.isSet() ) {
				if ( licenseArg.getValue() == hyfydyOptions[0] )
					std::cout << std::endl << "Hardware ID: " << scone::GetHardwareId() << std::endl;
				else if ( licenseArg.getValue() == hyfydyOptions[1] )
					scone::AddLicenseInteractive();
			}
#endif
		}
		catch ( std::exception& e )
		{
			scone::log::Critical( e.what() );
			if ( statusOutput.isSet() )
			{
				std::cout << std::endl << "*error=" << xo::try_quoted( e.what() ) << std::endl;
				std::cout.flush();
				xo::sleep( 5000 );
			}
		}
	}
	catch ( std::exception& e )
	{
		scone::log::critical( e.what() );
		return -1;
	}
	catch ( TCLAP::ExitException& e )
	{
		return e.getExitStatus();
	}

	return 0;
}
