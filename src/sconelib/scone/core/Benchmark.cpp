#include "Benchmark.h"

#include "scone/core/types.h"
#include "scone/core/Factories.h"
#include "scone/optimization/Optimizer.h"
#include "scone/optimization/SimulationObjective.h"
#include "scone/core/profiler_config.h"

#include "xo/time/timer.h"
#include "xo/container/prop_node_tools.h"
#include "xo/filesystem/filesystem.h"
#include "xo/serialization/char_stream.h"
#include "xo/container/container_algorithms.h"
#include "xo/time/time.h"
#include "xo/thread/thread_priority.h"
#include "Log.h"

namespace scone
{
	void BenchmarkScenario( const PropNode& scenario_pn, const path& file, const path& results_dir, size_t evals )
	{
		log::info( file.parent_path().stem() / file.filename() );

		xo::scoped_thread_priority prio_raiser( xo::thread_priority::realtime );

		auto opt = CreateOptimizer( scenario_pn, file.parent_path() );
		auto mo = dynamic_cast<ModelObjective*>( &opt->GetObjective() );
		auto par = SearchPoint( mo->info() );
		bool is_par_file = file.extension_no_dot() == "par";
		if ( is_par_file )
			par.import_values( file );

		auto baseline_file = results_dir / xo::get_computer_name() / file.stem() + ".stats";
		bool has_baseline = xo::file_exists( baseline_file );
		if ( !has_baseline )
			evals *= 4;

		// run simulations
		xo::flat_map<string, std::vector<xo::time>> bm_components;
		xo::flat_map<string, std::vector<xo::time>> bm_totals;
		xo::time duration;
		for ( index_t idx = 0; idx < evals; ++idx )
		{
			xo::sleep( 100 ); // this sleep makes the benchmarks slightly more consistent (albeit slower) on Win64

			xo::timer t;
			auto model = mo->CreateModelFromParams( par );
			model->SetStoreData( false );
			auto create_model_time = t();
			mo->AdvanceSimulationTo( *model, model->GetSimulationEndTime() );
			auto total_time = t();
			auto timings = model->GetBenchmarks();
			if ( !timings.empty() )
			{
				for ( const auto& timing : timings )
					bm_components[ timing.first ].push_back( timing.second.first / timing.second.second );
				bm_components[ "EvalTotal" ].push_back( total_time );
				bm_components[ "EvalSim" ].push_back( ( total_time - create_model_time ) );
				if ( !timings.empty() )
					bm_components[ "EvalSimModel" ].push_back( timings.front().second.first );
				duration = xo::time_from_seconds( model->GetTime() );
				log::info( "Benchmarked trial ", idx + 1, " of ", evals, "; simulated ", duration, "s in ", total_time, "s (", duration / total_time, "x real-time)" );
			}
		}

		// read baseline
		xo::flat_map<string, xo::time> baseline_medians;
		if ( has_baseline )
		{
			xo::char_stream bstr( load_string( baseline_file ) );
			while ( bstr.good() )
			{
				string bname;
				double bmedian, bstd;
				bstr >> bname >> bmedian >> bstd;
				bool eval = xo::str_begins_with( bname, "Eval" );
				if ( bstr.good() )
					baseline_medians[ bname ] = eval ? xo::time_from_milliseconds( bmedian ) : xo::time_from_nanoseconds( bmedian );
			}
		}

		// process
		std::vector<Benchmark> benchmarks;
		for ( const auto& bms : bm_components )
		{
			Benchmark bm;
			bm.name_ = bms.first;
			bm.time_ = xo::median( bms.second );
			bm.baseline_ = baseline_medians[ bms.first ];
			bm.std_ = xo::mean_std( bms.second ).second;
			benchmarks.push_back( bm );
		}

		std::sort( benchmarks.begin(), benchmarks.end(), [&]( auto&& a, auto&& b ) { return a.time_ > b.time_; } );

		// report
		for ( const auto& bm : benchmarks )
		{
			bool eval = xo::str_begins_with( bm.name_, "Eval" );
			log::level l = bm.diff_std() > 1 ? log::level::error : ( bm.diff_std() < -1 ? log::level::warning : log::level::info );

			if ( eval )
				log::message( l, xo::stringf( "%-32s\t%5.0fms\t%+5.0fms\t%+6.2f%%\t%+6.2fS\t%6.2f\t(%.2fx real-time)", bm.name_.c_str(),
					bm.time_.milliseconds(), bm.diff().milliseconds(), bm.diff_perc(), bm.diff_std(), bm.std_ * 1e-6, duration / bm.time_ ) );
			else
				log::message( l, xo::stringf( "%-32s\t%5.0fns\t%+5.0fns\t%+6.2f%%\t%+6.2fS\t%6.2f", bm.name_.c_str(),
					bm.time_.nanosecondsd(), bm.diff().nanosecondsd(), bm.diff_perc(), bm.diff_std(), bm.std_ ) );

			if ( !has_baseline )
			{
				xo::create_directories( baseline_file.parent_path().str() );
				auto ostr = std::ofstream( baseline_file.str(), std::ios_base::app );
				if ( eval )
					ostr << xo::stringf( "%-32s\t%8.2f\t%8.2f\n", bm.name_.c_str(), bm.time_.milliseconds(), bm.std_ * 1e-6 );
				else
					ostr << xo::stringf( "%-32s\t%8.0f\t%8.2f\n", bm.name_.c_str(), bm.time_.nanosecondsd(), bm.std_ );
			}
		}

		if ( !has_baseline )
			log::info( "Performance results written to ", baseline_file );
	}
}
