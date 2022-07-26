/*
** CmaOptimizer.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "CmaOptimizerSpot.h"

#include "scone/core/Exception.h"
#include "scone/optimization/opt_tools.h"

#include "spot/stop_condition.h"
#include "spot/file_reporter.h"
#include "scone/core/Settings.h"

namespace scone
{
	CmaOptimizer::CmaOptimizer( const PropNode& pn, const PropNode& scenario_pn, const path& scenario_dir ) :
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
		if ( target_fitness_ == target_fitness_ )
			add_stop_condition( std::make_unique< spot::target_fitness_condition>( target_fitness_ ) );
	}

	void CmaOptimizer::SetOutputMode( OutputMode m )
	{
		xo_assert( output_mode_ == no_output ); // output mode can only be set once
		output_mode_ = m;
		if ( auto p = MakeSpotReporter( output_mode_ ) )
			add_reporter( std::move( p ) );
	}

	void CmaOptimizer::Run()
	{
		// create output folder
		PrepareOutputFolder();

		// create file reporter
		auto fr = std::make_unique< spot::file_reporter >( GetOutputFolder(), min_improvement_for_file_output, max_generations_without_file_output );
		fr->output_fitness_history_ = GetSconeSetting<bool>( "optimizer.output_fitness_history" );
		fr->output_par_history_ = GetSconeSetting<bool>( "optimizer.output_par_history" );

		add_reporter( std::move( fr ) );

		run();
	}
}
