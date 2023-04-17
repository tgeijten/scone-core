#pragma once

#include "platform.h"
#include "xo/time/time.h"
#include "PropNode.h"
#include "xo/filesystem/path.h"
#include "types.h"

namespace scone
{
	struct BenchmarkOptions {
		size_t min_samples = 8;
		double min_norm_std = 0.01;
		bool log_history = true;
	};

	SCONE_API void BenchmarkScenario( 
		const PropNode& scenario_pn, const path& file, const path& results_dir, const BenchmarkOptions& opt );

	struct SCONE_API Benchmark {
		String name_;
		xo::time time_;
		xo::time baseline_;
		double std_;

		xo::time diff() const { return time_ - baseline_; }
		double diff_norm() const { return diff() / baseline_; }
		double diff_perc() const { return 100 * ( diff() / baseline_ ); }
		double diff_factor() const { return baseline_ / time_; }
		double diff_std() const { return diff().secondsd() / std_; }
	};
}
