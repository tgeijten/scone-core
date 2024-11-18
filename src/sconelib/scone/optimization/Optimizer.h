/*
** Optimizer.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Objective.h"
#include "Params.h"
#include "scone/core/HasSignature.h"
#include "scone/core/types.h"
#include "xo/system/log_sink.h"
#include <deque>
#include <mutex>
#include "ParInitSettings.h"
#include <thread>
#include <future>

namespace scone
{
	/// Base class for Optimizers.
	class SCONE_API Optimizer : public HasSignature
	{
	public:
		Optimizer( const PropNode& props, const PropNode& scenario_pn, const path& scenario_dir );
		Optimizer( const Optimizer& ) = delete;
		Optimizer& operator=( const Optimizer& ) = delete;

		virtual ~Optimizer();

		/// Parameter file (.par) to be used for initial parameter values.
		path init_file;

		/// Use init_file (if exists); default = true.
		bool use_init_file;

		/// Use the standard deviations from the init_file; when set to false, the initial standard deviation is 
		/// computed as follows: STDEV = parameter value * init_file_std_factor + init_file_std_offset; default = true.
		bool use_init_file_std;

		/// Use values from the first column of the .par file (generation best) as mean; default = false.
		bool use_init_file_best_as_mean;

		/// Factor by which to multiply the standard deviations from the init_file; default = 1.0.
		double init_file_std_factor;

		/// Offset added to the standard deviations from init_file; default = 0.
		double init_file_std_offset;

		/// Pattern matching the init_file parameters to include (semicolon seperated); default = "" (all).
		String init_file_include;

		/// Pattern matching the init_file parameters to exclude (semicolon seperated); default = "" (none).
		String init_file_exclude;

		/// Section with parameter initialization settings (.par), multiple allowed; see ParInitSettings for details.
		ParInitSettings init;

		/// Maximum number of threads to use for this optimization (deprecated, use global settings instead); default = 32.
		size_t max_threads;

		/// Thread priority of the optimization; 0 = lowest, 7 = highest, default = 1.
		int thread_priority;

		/// Number of iterations after which to stop the optimization; default = 100000.
		size_t max_generations;

		/// Window size used for measuring progress; default = 500.
		size_t window_size;

		/// Minimum progress after which to stop the optimization; default = 1e-5.
		double min_progress;

		/// Minimum number of samples after which progress is measured; default = window_size.
		size_t min_progress_samples;

		/// The minimum relative improvement needed for file output; default = 0.05.
		Real min_improvement_for_file_output;

		/// The maximum number of iterations without file output; default = 1000.
		size_t max_generations_without_file_output;

		/// Target fitness value, stop optimization if better; default = not set.
		double target_fitness_;

		/// Set the log level [1-7] for the optimization.log file (higher is less logging); default = 3.
		xo::log::level log_level_;

		Objective& GetObjective() { return *m_Objective; }
		const Objective& GetObjective() const { return *m_Objective; }

		// get the results output folder
		const path& GetOutputFolder() const;

		virtual bool IsMinimizing() const { return m_Objective->info().minimize(); }

		virtual double GetBestFitness() const = 0;

		// #todo: move this to reporter
		enum OutputMode { no_output, console_output, status_console_output, status_queue_output };
		virtual void SetOutputMode( OutputMode m ) { output_mode_ = m; }
		bool GetStatusOutput() const { return output_mode_ == status_console_output || output_mode_ == status_queue_output; }
		PropNode GetStatusPropNode() const;
		void OutputStatus( PropNode&& pn ) const;
		template< typename T > void OutputStatus( const String& key, const T& value ) const;
		std::deque<PropNode> GetStatusMessages() const;

		const String& id() const { return id_; }

		mutable size_t m_LastFileOutputGen;
		path output_root;
		bool show_optimization_time;
		bool output_objective_result_files;

		void PrepareOutputFolder();
		void Run();
		void RunBackground();
		size_t GetCurrentStep() const;
		bool WaitToFinish( int timeout_ms = -1 ) const;
		bool IsFinished() const;
		void Terminate();

	protected:
		ObjectiveUP m_Objective;
		virtual String GetClassSignature() const override;
		virtual void RunImpl() = 0;

		OutputMode output_mode_;
		mutable std::deque<PropNode> status_queue_; // #todo: move this to reporter
		mutable std::mutex status_queue_mutex_;

		mutable path output_folder_;
		mutable String id_;

		u_ptr< xo::log::file_sink > log_sink_;

		PropNode scenario_pn_copy_; // copy for creating props in output folder

		std::future<void> future_;
	};

	template< typename T >
	void scone::Optimizer::OutputStatus( const String& key, const T& value ) const
	{
		PropNode pn = GetStatusPropNode();
		pn.set( key, value );
		OutputStatus( std::move( pn ) );
	}
}
