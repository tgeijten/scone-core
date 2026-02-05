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
	void BenchmarkScenario( const PropNode& scenario_pn, const path& file, const BenchmarkOptions& bo )
	{
		string bench_name = file.stem().str();
		log::info( "---\nBENCHMARK: ", file.parent_path().stem() / file.filename() );

		// read baseline
		bool has_baseline = false;
		string baseline_result_str;
		xo::flat_map<string, double> baseline_rtx;
		if ( xo::file_exists( bo.baseline_file ) ) {
			auto lines = xo::split_str( xo::load_string( bo.baseline_file ), "\r\n" );
			SCONE_ASSERT( lines.size() > 0 );
			auto headers = xo::split_str( lines.front(), " \t" );
			SCONE_ASSERT( headers.size() > 3 );
			for ( auto& line : lines ) {
				auto values = xo::split_str( line, " \t" );
				if ( values.size() == headers.size() && values.front() == bench_name ) {
					has_baseline = true;
					baseline_result_str = values[2];
					for ( size_t i = 3; i < values.size(); ++i )
						baseline_rtx[headers[i]] = std::stod( values[i] );
				}
			}
		}

		xo::scoped_thread_priority prio_raiser( xo::thread_priority::realtime );

		auto opt = CreateOptimizer( scenario_pn, file.parent_path() );
		auto mo = dynamic_cast<ModelObjective*>( &opt->GetObjective() );
		auto par = SearchPoint( mo->info() );
		bool is_par_file = file.extension_no_dot() == "par";
		if ( is_par_file )
			par.import_values( file );

		auto min_samples = bo.min_samples;
		auto max_samples = min_samples * 20;

		// run simulations
		xo::flat_map<string, std::vector<TimeInSeconds>> bm_components;
		auto add_benchmark = [&]( const std::string sv, xo::time value ) {
			auto& bm = bm_components[sv];
			if ( bm.empty() )
				bm.reserve( max_samples );
			bm.push_back( value.secondsd() );
		};
		xo::time duration;
		double result = 0.0;
		std::vector<TimeInSeconds> bmsortedbest;
		index_t samples = 0;
		for ( ; samples < max_samples; ++samples )
		{
			xo::timer t;
			auto model = mo->CreateModelFromParams( par );
			model->SetStoreData( false );
			auto create_model_time = t();
			mo->AdvanceSimulationTo( *model, model->GetSimulationEndTime() );
			auto total_time = t();
			auto timings = model->GetBenchmarks();
			result = model->GetMeasureResult();
			if ( !timings.empty() )
			{
				for ( const auto& timing : timings )
					add_benchmark( timing.first, timing.second.first / timing.second.second );
				add_benchmark( "EvalTotal", total_time );
				add_benchmark( "EvalSim", ( total_time - create_model_time ) );
				add_benchmark( "EvalEngine", timings.front().second.first );
				duration = xo::time_from_seconds( model->GetTime() );
				auto real_time_x = model->GetTime() / total_time.secondsd();

				auto& bmvec = bm_components["EvalTotal"];
				//auto n = ( bmvec.size() + 1 ) / 2;
				auto n = std::min( min_samples, bmvec.size() );
				bmsortedbest.resize( n );
				std::partial_sort_copy( bmvec.begin(), bmvec.end(), bmsortedbest.begin(), bmsortedbest.end() );
				auto [mean, stdev] = xo::mean_std( bmsortedbest );
				auto rt_mean = model->GetTime() / mean;
				auto norm_std = stdev / mean;
				printf( "%03zd: %6.2f M=%6.2f S=%.4f\r", samples, real_time_x, rt_mean, norm_std );
				if ( norm_std < bo.min_norm_std && samples >= min_samples )
					break;
			}
			//xo::sleep( 100 ); // this sleep makes the benchmarks slightly more consistent (albeit slower) on Win64
		}

		if ( samples == max_samples )
			log::error( "Maximum number of samples was reached, results may be inaccurate" );

		// process
		std::vector<Benchmark> benchmarks;
		for ( const auto& [name, samples] : bm_components )
		{
			Benchmark bm;
			bm.name_ = name;
			std::partial_sort_copy( samples.begin(), samples.end(), bmsortedbest.begin(), bmsortedbest.end() );
			auto [mean, stdev] = xo::mean_std( bmsortedbest );
			bm.time_ = xo::time_from_seconds( mean );
			bm.baseline_ = has_baseline ? duration / baseline_rtx[name] : duration;
			bm.std_ = stdev;
			benchmarks.push_back( bm );
		}

		std::sort( benchmarks.begin(), benchmarks.end(), [&]( auto&& a, auto&& b ) { return a.time_ > b.time_; } );

		// report
		auto result_str = xo::to_str( result );
		const auto simulator_id = mo->GetModel().GetSimulatorId();
		const char* fmt = "%-32s\t%-20s\t%-8s";
		string results_header = xo::stringf( fmt, "Benchmark", "SimulatorId", "Result" );
		string results_string = xo::stringf( fmt, bench_name.c_str(), simulator_id.c_str(), result_str.c_str() );
		for ( const auto& bm : benchmarks )
		{
			bool eval = xo::str_begins_with( bm.name_, "Eval" );
			auto diff_std = bm.diff_norm() / bo.min_norm_std;
			log::level l = diff_std > 3 ? log::level::error : ( diff_std < -3 ? log::level::warning : log::level::info );

			if ( eval )
			{
				log::message( l, xo::stringf( "%-32s\t%5.0fms\t%+5.0fms\t%5.3fx\t%+6.2fS\t%6.4f\t(%.2fx real-time)", bm.name_.c_str(),
					bm.time_.milliseconds(), bm.diff().milliseconds(), bm.diff_factor(), diff_std, bm.std_, duration / bm.time_ ) );
				results_string += xo::stringf( "\t%12.3f", duration / bm.time_ );
				results_header += xo::stringf( "\t%12s", bm.name_.c_str() );
			}
			else
				log::info( xo::stringf( "%-32s\t%5.0fns", bm.name_.c_str(), bm.time_.nanosecondsd() ) );
		}

		log::info( "result=", result, " duration=", duration.secondsd(), " samples=", samples );
		if ( has_baseline && baseline_result_str != result_str )
			log::error( "Result is different from baseline: ", result_str, " != ", baseline_result_str );


		if ( !bo.results_file.empty() ) {
			path fixed_results_file = bo.results_file;
			fixed_results_file.concat_stem( "_" + xo::remove_strs( simulator_id, { "Hyfydy-", "-DBL" } ) );
			if ( !xo::file_exists( fixed_results_file ) )
				xo::save_string( fixed_results_file, results_header + "\n" );
			xo::append_string( fixed_results_file, results_string + "\n" );
		}
	}
}
