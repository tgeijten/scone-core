/*
** sconecmd.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
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

int main( int argc, char* argv[] )
{
	xo::log::console_sink console_sink( xo::log::level::info );
	xo::log::info( "SCONE version ", scone::GetSconeVersion() );

	try
	{
		scone::Initialize();
		auto filename = scone::GetDataFolder() / "Benchmarks";
		if ( xo::directory_exists( filename ) )
		{
			auto files = xo::find_files( filename, "bench*.scone", true, 0 );
			for ( const auto& f : files )
			{
				auto scenario_pn = scone::LoadScenario( scone::FindScenario( f ), false );
				scone::BenchmarkScenario( scenario_pn, f, filename / "_benchmark_results", 10 );
			}
		}
	} catch ( std::exception& e )
	{
		scone::log::critical( e.what() );
		return -1;
	}
	return 0;
}
