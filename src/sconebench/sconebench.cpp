/*
** sconebench.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "scone/core/Exception.h"
#include "scone/core/Factories.h"
#include "scone/core/Log.h"
#include "scone/core/version.h"
#include "scone/core/Benchmark.h"
#include "xo/system/log.h"
#include "xo/system/log_sink.h"
#include "scone/sconelib_config.h"
#include "xo/filesystem/filesystem.h"
#include "scone/optimization/opt_tools.h"
#include "xo/utility/arg_parser.h"
#include "xo/string/pattern_matcher.h"

int main( int argc, const char* argv[] )
{
	xo::log::console_sink console_sink( xo::log::level::info );

	try
	{
		auto args = xo::arg_parser( argc, argv );
		console_sink.set_log_level( xo::log::level( args.get<int>( "l", int( xo::log::level::info ) ) ) );

		xo::log::info( "SCONE version ", scone::GetSconeVersion() );
		scone::Initialize();

		xo::pattern_matcher include = args.get<std::string>( "include", "bench*.scone" );
		bool fast = args.has_flag( "fast" );
		auto min_samples = args.get<size_t>( "min_samples", fast ? 8 : 12 );
		auto min_norm_std = args.get<double>( "min_norm_std", fast ? 0.05 : 0.01 );
		auto folder = scone::GetFolder( scone::SconeFolder::Scenarios ) / args.get<std::string>( 0, "Benchmarks" );
		xo::log::info( "Running benchmarks from ", folder );
		if ( xo::directory_exists( folder ) )
		{
			auto files = xo::find_files( folder, include, true, 0 );
			for ( const auto& f : files )
			{
				try {
					auto scenario_pn = scone::LoadScenario( f );
					scone::BenchmarkScenario( scenario_pn, f, folder / "_benchmark_results", min_samples, min_norm_std );
				}
				catch ( std::exception& e ) {
					scone::log::error( "Error benchmarking ", f.filename(), ": ", e.what() );
				}
			}
		}
	}
	catch ( std::exception& e ) {
		scone::log::critical( e.what() );
		return -1;
	}
	return 0;
}
