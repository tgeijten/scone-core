/*
** CmaOptimizerSpot.h
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "EsOptimizer.h"
#include "spot/cma_optimizer.h"

namespace scone
{
	using spot::optimizer;
	using spot::search_point;
	using spot::search_point_vec;
	using spot::fitness_vec;
	using xo::index_t;

	/// Optimizer based on the CMA-ES algorithm by [Hansen].
	class SCONE_API CmaOptimizerSpot : public EsOptimizer, public spot::cma_optimizer
	{
	public:
		CmaOptimizerSpot( const PropNode& pn, const PropNode& scenario_pn, const path& scenario_dir );
		virtual void SetOutputMode( OutputMode m ) override;
		virtual ~CmaOptimizerSpot() = default;
		virtual void Run() override;
		virtual double GetBestFitness() const override { return best_fitness(); }

		/// Maximum number of errors allowed during evaluation, use a negative value equates to ''lambda - max_errors''; default = 0
		int max_errors; // for documentation only, copies value to spot::max_errors_ during construction
	};
}
