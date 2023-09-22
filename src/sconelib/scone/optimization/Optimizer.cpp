/*
** Optimizer.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Optimizer.h"

#include "scone/core/Log.h"
#include "scone/core/system_tools.h"
#include "scone/core/string_tools.h"
#include "scone/core/Factories.h"
#include "scone/core/math.h"
#include "scone/optimization/Objective.h"
#include "scone/optimization/ModelObjective.h"
#include "scone/optimization/opt_tools.h"

#include "xo/filesystem/filesystem.h"
#include "xo/container/prop_node_tools.h"
#include "xo/system/system_tools.h"
#include "xo/serialization/serialize.h"
#include "xo/serialization/prop_node_serializer_zml.h"
#include "xo/system/error_code.h"
#include "xo/thread/thread_priority.h"

#include <mutex>
#include <sstream>
#include <limits>

namespace scone
{
	std::mutex g_status_output_mutex;

	Optimizer::Optimizer( const PropNode& props, const PropNode& scenario_pn, const path& scenario_dir ) :
		HasSignature( props ),
		max_threads( 1 ),
		thread_priority( (int)xo::thread_priority::lowest ),
		m_LastFileOutputGen( 0 ),
		m_Objective( CreateObjective( FindFactoryProps( GetObjectiveFactory(), props, "Objective" ), scenario_dir ) ),
		m_BestFitness( m_Objective->info().worst_fitness() ),
		output_mode_( no_output ),
		scenario_pn_copy_( scenario_pn )
	{
		INIT_PROP( props, output_root, GetFolder( SconeFolder::Results ) );
		log_level_ = static_cast<xo::log::level>( props.get<int>( "log_level", (int)xo::log::level::info ) );

		INIT_PROP( props, max_threads, size_t( 32 ) );
		INIT_PROP( props, thread_priority, (int)xo::thread_priority::lowest );
		INIT_PROP( props, show_optimization_time, false );

		INIT_PROP( props, init_file, path( "" ) );
		INIT_PROP( props, use_init_file, true );
		INIT_PROP( props, init_file_std_factor, 1.0 );
		INIT_PROP( props, init_file_std_offset, 0.0 );
		INIT_PROP( props, use_init_file_std, true );
		INIT_PROP( props, use_init_file_best_as_mean, false );
		INIT_PROP( props, init_file_include, {} );
		INIT_PROP( props, init_file_exclude, {} );

		INIT_PROP( props, output_objective_result_files, false );
		INIT_PROP( props, min_improvement_for_file_output, 0.05 );
		INIT_PROP( props, max_generations_without_file_output, 1000 );

		INIT_PROP( props, max_generations, 100000 );

		INIT_PROP( props, min_progress, 1e-5 );
		INIT_PROP( props, window_size, 500 );
		INIT_PROP( props, min_progress_samples, window_size );

		INIT_PROP( props, target_fitness_, std::numeric_limits<double>::quiet_NaN() );

		// initialize parameters from init_file
		auto& info = GetObjective().info();
		if ( use_init_file && !init_file.empty() )
		{
			init_file = FindFile( init_file );
			auto result = info.import_mean_std( init_file,
				use_init_file_std, init_file_std_factor, init_file_std_offset,
				init_file_include, init_file_exclude, use_init_file_best_as_mean );
			log::debug( "Imported ", result.first, " of ", info.dim(), ", skipped ", result.second, " parameters from ", init_file );
		}

		// initialize parameters from init sections (possibly multiple)
		for ( auto& [key, init_pn] : props.select( "init" ) ) {
			auto init = ParInitSettings( init_pn );
			init.file = FindFile( init.file );
			GetObjective().AddExternalResource( init.file );
			std::pair< size_t, size_t > r;
			if ( !init.locked )
				r = info.import_mean_std( init.file, init.use_std, init.std_factor, init.std_offset, init.include, init.exclude, init.use_best_as_mean );
			else r = info.import_locked( init.file );
			log::debug( "Imported ", r.first, " of ", info.dim(), ", skipped ", r.second, " parameters from ", init_file );
		}

		// inject model build version if this is a model objective
		if ( const auto* mo = dynamic_cast<ModelObjective*>( m_Objective.get() ) )
			if ( auto* model_pn = TryGetModelPropNode( scenario_pn_copy_ ) )
				mo->GetModel().AddVersionToPropNode( *model_pn );
	}

	Optimizer::~Optimizer()
	{}

	const path& Optimizer::GetOutputFolder() const
	{
		SCONE_ASSERT( !output_folder_.empty() );
		return output_folder_;
	}

	PropNode Optimizer::GetStatusPropNode() const
	{
		PropNode pn;
		if ( !id_.empty() ) pn["id"] = id_;
		return pn;
	}

	void Optimizer::OutputStatus( PropNode&& pn ) const
	{
		if ( output_mode_ == status_console_output )
		{
			xo::error_code ec;
			std::ostringstream str;
			str << xo::prop_node_serializer_zml_concise( pn, &ec );
			auto message = "*" + str.str();
			g_status_output_mutex.lock();
			std::cout << message << std::endl;
			g_status_output_mutex.unlock();
		}
		else if ( output_mode_ == status_queue_output )
		{
			auto lock = std::scoped_lock( status_queue_mutex_ );
			status_queue_.push_back( std::move( pn ) );
		}
	}

	std::deque<PropNode> Optimizer::GetStatusMessages() const
	{
		std::deque<PropNode> results;
		{
			auto lock = std::scoped_lock( status_queue_mutex_ );
			if ( !status_queue_.empty() )
			{
				results = std::move( status_queue_ );
				status_queue_.clear();
			}
		}
		return results;
	}

	String Optimizer::GetClassSignature() const
	{
		String s = GetObjective().GetSignature();
		if ( use_init_file && !init_file.empty() )
			s += ".I";

		return s;
	}

	void Optimizer::PrepareOutputFolder()
	{
		SCONE_ERROR_IF( GetObjective().dim() <= 0, "Objective has no free parameters" );
		SCONE_ASSERT( output_folder_.empty() );

		auto p = output_root / GetSignature();
		log::debug( "Creating folder ", p );
		output_folder_ = xo::create_unique_directory( p );
		id_ = output_folder_.filename().str();

		// create log sink if enabled
		if ( log_level_ < xo::log::level::never )
			log_sink_ = std::make_unique<xo::log::file_sink>(
				GetOutputFolder() / "optimization.log", log_level_, xo::log::sink_mode::current_thread );

		// prepare output folder, and initialize
		xo::save_file( scenario_pn_copy_, output_folder_ / "config.scone" );
		if ( use_init_file && !init_file.empty() )
			xo::copy_file( init_file, output_folder_ / init_file.filename(), true );

		// copy all objective resources to output folder
		xo::error_code ec;
		for ( auto& f : GetObjective().GetExternalResources() )
			if ( !xo::copy_file( f, output_folder_ / f.filename(), true, &ec ) )
				SCONE_ERROR( "Could not copy \"" + f.str() +
					"\" to \"" + ( output_folder_ / f.filename() ).str() + "\"\n\n" + ec.message() );

		// now that all files are copied, we should use these during evaluation
		GetObjective().SetExternalResourceDir( GetOutputFolder() );
	}
}
