/*
** opt_tools.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "opt_tools.h"
#include "scone/core/types.h"
#include "scone/core/Factories.h"
#include "scone/optimization/SimulationObjective.h"
#include "scone/core/profiler_config.h"

#include "xo/time/timer.h"
#include "xo/container/prop_node_tools.h"
#include "xo/filesystem/filesystem.h"
#include "xo/serialization/char_stream.h"
#include "xo/utility/irange.h"
#include "xo/container/container_algorithms.h"
#include "xo/serialization/serialize.h"
#include "scone/core/Settings.h"
#include "xo/thread/thread_priority.h"
#include "spot/batch_evaluator.h"
#include "spot/async_evaluator.h"
#include "spot/pooled_evaluator.h"
#include "EsOptimizer.h"
#include "spot/console_reporter.h"

using xo::timer;

namespace scone
{
	bool LogUnusedProperties( const PropNode& pn )
	{
		// report unused properties
		if ( pn.count_unaccessed() > 0 )
		{
			log::warning( "Warning, unused properties:" );
			xo::log_unaccessed( pn );
			return true;
		}
		else return false;
	}

	PropNode EvaluateScenario( const PropNode& scenario_pn, const path& par_file, const path& output_base )
	{
		bool store_data = !output_base.empty();

		auto opt = CreateOptimizer( scenario_pn, par_file.parent_path() );
		auto mo = dynamic_cast<ModelObjective*>( &opt->GetObjective() );

		// report unused properties
		LogUnusedProperties( scenario_pn );

		// create model
		bool has_par_file = par_file.extension_no_dot() == "par";
		ModelUP model = has_par_file ? mo->CreateModelFromParFile( par_file ) : mo->CreateModelFromParams( mo->info() );

		model->SetStoreData( store_data );

		timer tmr;
		auto result = mo->EvaluateModel( *model, xo::stop_token() );
		auto duration = tmr().secondsd();

		// write results
		if ( store_data )
		{
			auto files = model->WriteResults( output_base );
			log::info( "Results written to " + output_base.str() + "*" );
		}

		// collect statistics
		PropNode statistics;
		statistics.set( "result", mo->GetReport( *model ) );
		statistics.set( "simulation time", model->GetTime() );
		statistics.set( "performance (x real-time)", model->GetTime() / duration );

		return statistics;
	}

	path FindScenario( const path& file )
	{
		if ( file.extension_no_dot() == "scone" || file.extension_no_dot() == "xml" )
			return file;
		auto folder = file.parent_path();
		return xo::find_file( { path( file ).replace_extension( "scone" ), folder / "config.scone", folder / "config.xml" } );
	}

	PropNode* TryGetModelPropNode( PropNode& scenario_pn )
	{
		if ( auto opt_fp = FindFactoryProps( GetOptimizerFactory(), scenario_pn, "Optimizer" ) )
			if ( auto obj_fp = FindFactoryProps( GetObjectiveFactory(), opt_fp.props(), "Objective" ) )
				if ( auto mod_fp = FindFactoryProps( GetModelFactory(), obj_fp.props(), "Model" ) )
					return &const_cast<PropNode&>( mod_fp.props() );
		return nullptr;
	}

	void AddEmptyVersionForOldScenarios( PropNode& scenario_pn )
	{
		if ( auto* pn = TryGetModelPropNode( scenario_pn ) )
		{
			if ( !pn->has_key( "scone_version" ) )
				pn->add_key_value( "scone_version", xo::version() );
		}
		else log::warning( "Could not find Model in scenario" );
	}

	PropNode LoadScenario( const path& scenario_file, bool add_missing_version )
	{
		PropNode scenario_pn;
		scenario_pn = xo::load_file_with_include( scenario_file, "INCLUDE" );
		if ( add_missing_version )
			AddEmptyVersionForOldScenarios( scenario_pn );
		return scenario_pn;
	}

	spot::evaluator& GetSpotEvaluator()
	{
		auto eval = GetSconeSetting<int>( "optimizer.evaluator" );
		auto max_threads = GetSconeSetting<int>( "optimizer.max_threads" );
		auto thread_prio = static_cast<xo::thread_priority>( GetSconeSetting<int>( "optimizer.thread_priority" ) );
		if ( eval == 0 )
		{
			static spot::sequential_evaluator sequential_eval;
			return sequential_eval;
		}
		else if ( eval == 1 )
		{
			static spot::batch_evaluator batch_eval;
			return batch_eval;
		}
		else if ( eval == 2 )
		{
			static spot::async_evaluator async_eval( max_threads );
			async_eval.set_max_threads( max_threads, thread_prio );
			return async_eval;
		}
		else if ( eval == 3 )
		{
			static spot::pooled_evaluator pooled_eval( max_threads, thread_prio );
			pooled_eval.set_max_threads( max_threads, thread_prio );
			return pooled_eval;
		}
		else SCONE_THROW( "Invalid evaluator setting" );
	}

	std::unique_ptr<spot::reporter> MakeSpotReporter( Optimizer::OutputMode m )
	{
		switch ( m )
		{
		case Optimizer::no_output:
			return nullptr;
		case Optimizer::console_output:
			return std::make_unique<spot::console_reporter>();
		case Optimizer::status_console_output:
		case Optimizer::status_queue_output:
			return std::make_unique<EsOptimizerReporter>();
		default: SCONE_THROW( "Unknown output mode" );
		}
	}
}
