/*
** EvaOptimizer.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "EsOptimizer.h"
#include "spot/eva_optimizer.h"

namespace scone
{
	class SCONE_API EvaOptimizer : public EsOptimizer, public spot::eva_optimizer
	{
	public:
		EvaOptimizer( const PropNode& pn, const PropNode& scenario_pn, const path& scenario_dir );
		virtual void SetOutputMode( OutputMode m ) override;
		virtual ~EvaOptimizer() = default;
		virtual void Run() override;
		virtual double GetBestFitness() const override { return best_fitness(); }

		/// Maximum number of errors allowed during evaluation, use a negative value equates to ''lambda - max_errors''; default = 0
		int max_errors; // for documentation only, copies value to spot::max_errors_ during construction
	};
}
