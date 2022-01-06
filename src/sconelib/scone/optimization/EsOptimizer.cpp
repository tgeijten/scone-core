/*
** CmaOptimizer.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "EsOptimizer.h"
#include "xo/string/string_tools.h"
#include "xo/filesystem/filesystem.h"
#include "spot/optimizer.h"
#include "scone/core/Log.h"

namespace scone
{
	const long DEFAULT_RANDOM_SEED = 123;

	EsOptimizer::EsOptimizer( const PropNode& props, const PropNode& scenario_pn, const path& scenario_dir ) :
		Optimizer( props, scenario_pn, scenario_dir ),
		mu_( 0 ),
		lambda_( 0 ),
		sigma_( 1.0 ),
		max_attempts( 100 )
	{
		INIT_PROP( props, lambda_, 0 );
		INIT_PROP( props, mu_, 0 );
		INIT_PROP( props, sigma_, 1.0 );
		INIT_PROP( props, random_seed, DEFAULT_RANDOM_SEED );
		INIT_PROP( props, flat_fitness_epsilon_, 1e-6 );
	}

	String EsOptimizer::GetClassSignature() const
	{
		auto str = Optimizer::GetClassSignature();
		if ( random_seed != DEFAULT_RANDOM_SEED )
			str += xo::stringf( ".R%d", random_seed );
		return str;
	}

	EsOptimizerReporter::EsOptimizerReporter() :
		number_of_evaluations_( 0 )
	{}

	void EsOptimizerReporter::on_start( const spot::optimizer& opt )
	{
		auto& cma = dynamic_cast<const EsOptimizer&>( opt );

		log::info( "Starting optimization ", cma.id(), " dim=", opt.info().dim(), " lambda=", cma.lambda_, " mu=", cma.mu_ );
		if ( cma.GetStatusOutput() )
		{
			PropNode pn = cma.GetStatusPropNode();
			pn.set( "folder", cma.GetOutputFolder() );
			pn.set( "dim", opt.info().dim() );
			pn.set( "sigma", cma.sigma_ );
			pn.set( "lambda", cma.lambda_ );
			pn.set( "mu", cma.mu_ );
			pn.set( "max_generations", cma.max_generations );
			pn.set( "minimize", cma.IsMinimizing() );
			pn.set( "window_size", cma.window_size );
			cma.OutputStatus( std::move( pn ) );
		}

		timer_.restart();
		number_of_evaluations_ = 0;
	}

	void EsOptimizerReporter::on_stop( const spot::optimizer& opt, const spot::stop_condition& s )
	{
		auto& es_opt = dynamic_cast<const EsOptimizer&>( opt );
		es_opt.OutputStatus( "finished", s.what() );
		log::info( "Optimization ", es_opt.id(), " finished: ", s.what() );
	}

	void EsOptimizerReporter::on_pre_evaluate_population( const spot::optimizer& opt, const spot::search_point_vec& pop )
	{
		auto& es_opt = dynamic_cast<const EsOptimizer&>( opt );
	}

	void EsOptimizerReporter::on_post_evaluate_population( const spot::optimizer& opt, const spot::search_point_vec& pop, const spot::fitness_vec& fitnesses, bool new_best )
	{
		auto& es_opt = dynamic_cast<const EsOptimizer&>( opt );

		number_of_evaluations_ += pop.size();
		auto t = timer_().secondsd();

		// report results
		auto pn = es_opt.GetStatusPropNode();
		pn.set( "step", opt.current_step() );
		pn.set( "step_best", opt.current_step_best_fitness() );
		pn.set( "step_median", xo::median( opt.current_step_fitnesses() ) );
		pn.set( "trend_offset", opt.fitness_trend().offset() );
		pn.set( "trend_slope", opt.fitness_trend().slope() );
		pn.set( "progress", opt.progress() );
		pn.set( "predicted_fitness", opt.predicted_fitness( opt.fitness_tracking_window_size() ) );
		pn.set( "time", t );
		pn.set( "number_of_evaluations", number_of_evaluations_ );
		pn.set( "evaluations_per_sec", number_of_evaluations_ / t );
		if ( new_best )
		{
			pn.set( "best", opt.best_fitness() );
			pn.set( "best_gen", opt.current_step() );
		}
		es_opt.OutputStatus( std::move( pn ) );
	}
}
