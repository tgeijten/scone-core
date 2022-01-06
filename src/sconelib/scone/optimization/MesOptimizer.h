/*
** MesOptimizer.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "EsOptimizer.h"
#include "spot/mes_optimizer.h"

namespace scone
{
	class SCONE_API MesOptimizer : public EsOptimizer, public spot::mes_optimizer
	{
	public:
		MesOptimizer( const PropNode& pn, const PropNode& scenario_pn, const path& scenario_dir );
		virtual void SetOutputMode( OutputMode m ) override;
		virtual ~MesOptimizer() = default;
		virtual void Run() override;
		virtual double GetBestFitness() const override { return best_fitness(); }

		/// Maximum number of errors allowed during evaluation, use a negative value equates to ''lambda - max_errors''; default = 0
		int max_errors; // for documentation only, copies value to spot::max_errors_ during construction
	};
}
