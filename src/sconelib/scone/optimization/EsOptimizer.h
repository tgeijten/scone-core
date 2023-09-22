/*
** CmaOptimizer.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Optimizer.h"
#include "scone/core/Exception.h"
#include "spot/reporter.h"
#include "xo/time/timer.h"

namespace scone
{
	/// Base class for optimizers that use Evolutionary Strategies (ES).
	class SCONE_API EsOptimizer : public Optimizer
	{
	public:
		EsOptimizer( const PropNode& props, const PropNode& scenario_pn, const path& scenario_dir );
		EsOptimizer( const EsOptimizer& ) = delete;
		EsOptimizer& operator=( const EsOptimizer& ) = delete;
		virtual ~EsOptimizer() = default;

		virtual void Run() override { SCONE_THROW( "Please use a subclass of EsOptimzer" ); }

		/// Lambda parameter (population size); default = automatic.
		int lambda_;

		/// Mu parameter (offspring size); default = automatic.
		int mu_;

		/// Sigma parameter (step size); default = automatic.
		double sigma_;

		/// Random seed used by the optimizer; default = 123.
		long random_seed;

		/// Epsilon value to detect flat fitness; default = 1e-6.
		double flat_fitness_epsilon_;

		/// Enable boundary transformer (experimental); default = 0.
		bool enable_boundary_transformer;

		int max_attempts;

	private: // non-copyable and non-assignable
		virtual String GetClassSignature() const override;
	};

	class SCONE_API EsOptimizerReporter : public spot::reporter
	{
	public:
		EsOptimizerReporter();
		virtual void on_start( const spot::optimizer& opt ) override;
		virtual void on_stop( const spot::optimizer& opt, const spot::stop_condition& s ) override;
		virtual void on_pre_evaluate_population( const spot::optimizer& opt, const spot::search_point_vec& pop ) override;
		virtual void on_post_evaluate_population( const spot::optimizer& opt, const spot::search_point_vec& pop, const spot::fitness_vec& fitnesses, bool new_best ) override;
		xo::timer timer_;
		size_t number_of_evaluations_;
	};
}
