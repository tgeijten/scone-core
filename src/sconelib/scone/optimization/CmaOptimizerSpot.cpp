/*
** CmaOptimizerSpot.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "CmaOptimizerSpot.h"

#include "spot/stop_condition.h"
#include "spot/file_reporter.h"
#include "spot/console_reporter.h"

#include "scone/core/Exception.h"
#include "scone/core/Log.h"
#include "scone/core/Settings.h"
#include "spot/async_evaluator.h"	
#include "spot/pooled_evaluator.h"
#include "spot/batch_evaluator.h"
#include "opt_tools.h"

namespace scone
{
	CmaOptimizerSpot::CmaOptimizerSpot( const PropNode& pn, const PropNode& scenario_pn, const path& scenario_dir ) :
		EsOptimizer( pn, scenario_pn, scenario_dir ),
		cma_optimizer( *m_Objective, GetSpotEvaluator(),
			spot::cma_options{
				EsOptimizer::lambda_,
				EsOptimizer::random_seed,
				spot::cma_weights::log,
				pn.get<double>( "update_eigen_modulo", -1.0 )
			}
		),
		INIT_MEMBER( pn, max_errors, max_errors_ )
	{
		SCONE_ASSERT( GetObjective().dim()  > 0 );

		max_errors_ = max_errors; // copy to spot::optimizer::max_errors_
		lambda_ = lambda();
		mu_ = mu();
		sigma_ = sigma();

		set_fitness_tracking_window_size( window_size );

		// stop conditions
		add_stop_condition( std::make_unique< spot::max_steps_condition >( max_generations ) );
		add_stop_condition( std::make_unique< spot::min_progress_condition >( min_progress, min_progress_samples ) );
		find_stop_condition< spot::flat_fitness_condition >().epsilon_ = flat_fitness_epsilon_;
	}

	void CmaOptimizerSpot::SetOutputMode( OutputMode m )
	{
		xo_assert( output_mode_ == no_output ); // output mode can only be set once
		output_mode_ = m;
		switch ( output_mode_ )
		{
		case Optimizer::no_output:
			break;
		case Optimizer::console_output:
			add_reporter( std::make_unique<spot::console_reporter>() );
			break;
		case Optimizer::status_console_output:
		case Optimizer::status_queue_output:
			add_reporter( std::make_unique<EsOptimizerReporter>() );
			break;
		default: SCONE_THROW( "Unknown output mode" );
		}
	}

	void CmaOptimizerSpot::Run()
	{
		// create output folder
		PrepareOutputFolder();

		// create file reporter
		add_reporter( std::make_unique< spot::file_reporter >(
			GetOutputFolder(), min_improvement_for_file_output, max_generations_without_file_output ) );

		run();
	}
}
