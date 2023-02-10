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
	void BenchmarkScenario( const PropNode& scenario_pn, const path& file, const path& results_dir, size_t min_samples, double min_norm_std )
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
			min_samples *= 4;
		auto max_samples = min_samples * 10;

		// run simulations
		xo::flat_map<string, std::vector<TimeInSeconds>> bm_components;
		auto add_benchmark = [&]( const std::string sv, xo::time value ) {
			auto& bm = bm_components[ sv ];
			if ( bm.empty() )
				bm.reserve( max_samples );
			bm.push_back( value.secondsd() );
		};
		xo::time duration;
		std::vector<TimeInSeconds> bmsortedbest;
		for ( index_t idx = 0; idx < max_samples; ++idx )
		{
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
					add_benchmark( timing.first, timing.second.first / timing.second.second );
				add_benchmark( "EvalTotal", total_time );
				add_benchmark( "EvalSim", ( total_time - create_model_time ) );
				add_benchmark( "EvalSimModel", timings.front().second.first );
				duration = xo::time_from_seconds( model->GetTime() );
				auto real_time_x = model->GetTime() / total_time.secondsd();

				auto& bmvec = bm_components[ "EvalTotal" ];
				//auto n = ( bmvec.size() + 1 ) / 2;
				auto n = std::min( min_samples, bmvec.size() );
				bmsortedbest.resize( n );
				std::partial_sort_copy( bmvec.begin(), bmvec.end(), bmsortedbest.begin(), bmsortedbest.end() );
				auto [mean, stdev] = xo::mean_std( bmsortedbest );
				auto rt_mean = model->GetTime() / mean;
				auto norm_std = stdev / mean;
				printf( "%03zd: %6.2f M=%6.2f S=%.4f\r", idx, real_time_x, rt_mean, norm_std );
				if ( norm_std < min_norm_std && idx >= min_samples )
					break;
			}
			//xo::sleep( 100 ); // this sleep makes the benchmarks slightly more consistent (albeit slower) on Win64
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
		for ( const auto& [name, samples] : bm_components )
		{
			Benchmark bm;
			bm.name_ = name;
			std::partial_sort_copy( samples.begin(), samples.end(), bmsortedbest.begin(), bmsortedbest.end() );
			auto [mean, stdev] = xo::mean_std( bmsortedbest );
			bm.time_ = xo::time_from_seconds( mean );
			bm.baseline_ = baseline_medians[ name ];
			bm.std_ = stdev;
			benchmarks.push_back( bm );
		}

		std::sort( benchmarks.begin(), benchmarks.end(), [&]( auto&& a, auto&& b ) { return a.time_ > b.time_; } );

		// report
		for ( const auto& bm : benchmarks )
		{
			bool eval = xo::str_begins_with( bm.name_, "Eval" );
			auto diff_std = bm.diff_norm() / min_norm_std;
			log::level l = diff_std > 3 ? log::level::error : ( diff_std < -3 ? log::level::warning : log::level::info );

			if ( eval )
				log::message( l, xo::stringf( "%-32s\t%5.0fms\t%+5.0fms\t%+6.2f%%\t%+6.2fS\t%6.2f\t(%.2fx real-time)", bm.name_.c_str(),
					bm.time_.milliseconds(), bm.diff().milliseconds(), bm.diff_perc(), diff_std, bm.std_, duration / bm.time_ ) );
			else
				log::message( l, xo::stringf( "%-32s\t%5.0fns\t%+5.0fns\t%+6.2f%%\t%+6.2fS\t%6.2f", bm.name_.c_str(),
					bm.time_.nanosecondsd(), bm.diff().nanosecondsd(), bm.diff_perc(), diff_std, bm.std_ ) );

			if ( !has_baseline )
			{
				xo::create_directories( baseline_file.parent_path().str() );
				auto ostr = std::ofstream( baseline_file.str(), std::ios_base::app );
				if ( eval )
					ostr << xo::stringf( "%-32s\t%8.2f\t%8.2f\n", bm.name_.c_str(), bm.time_.milliseconds(), bm.std_ );
				else
					ostr << xo::stringf( "%-32s\t%8.0f\t%8.2f\n", bm.name_.c_str(), bm.time_.nanosecondsd(), bm.std_ );
			}
		}
		log::info( "Simulation duration ", duration );

		if ( !has_baseline )
			log::info( "Results written to ", baseline_file );
	}
}
