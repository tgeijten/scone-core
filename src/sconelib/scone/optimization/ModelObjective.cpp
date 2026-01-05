/*
** ModelObjective.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "ModelObjective.h"

#include "scone/core/Factories.h"
#include "scone/core/Log.h"
#include "xo/filesystem/filesystem.h"
#include "opt_tools.h"
#include "scone/core/profiler_config.h"

namespace scone
{
	ModelObjective::ModelObjective( const PropNode& props, const path& find_file_folder ) :
		Objective( props, find_file_folder ),
		objective_pn_( props ),
		evaluation_step_size_( XO_IS_DEBUG_BUILD ? 0.01 : 0.25 )
	{
		// create internal model using the ORIGINAL prop_node to flag unused model props and create par_info_
		auto model_fp = FindFactoryProps( GetModelFactory(), props, "Model" );
		model_ = CreateModel( model_fp, info_, GetExternalResourceDir() );

		// create a controller that's defined OUTSIDE the model prop_node, using the ORIGINAL prop_node for flagging
		if ( auto controller_fp = TryFindFactoryProps( GetControllerFactory(), props, "Controller" ) )
			model_->CreateController( controller_fp, info_ );

		// create a measure that's defined OUTSIDE the model prop_node, using the ORIGINAL prop_node for flagging
		if ( auto measure_fp = TryFindFactoryProps( GetMeasureFactory(), props, "Measure" ) )
			model_->CreateMeasure( measure_fp, info_ );

		// now set FactoryProps for future use, using the COPY objective_pn_, since props may have been destructed
		model_factory_props_ = FindFactoryProps( GetModelFactory(), objective_pn_, "Model" );
		controller_factory_props_ = TryFindFactoryProps( GetControllerFactory(), objective_pn_, "Controller" );
		measure_factory_props_ = TryFindFactoryProps( GetMeasureFactory(), objective_pn_, "Measure" );

		// update the minimize flag in objective_info
		if ( model_->GetMeasure() )
			info_.set_minimize( model_->GetMeasure()->GetMinimize() );

		if ( info_.dim() > 0 && !model_->GetMeasure() )
			log::warning( "Warning: Model has free parameters but no Measure" );

		signature_ = model_->GetSignature();

		external_resources_.Add( model_->GetExternalResources() );
	}

	result<fitness_t> ModelObjective::evaluate( const SearchPoint& point, const xo::stop_token& st ) const
	{
		if ( !st.stop_requested() )
		{
			SearchPoint params( point );
			auto model = CreateModelFromParams( params );
			return EvaluateModel( *model, st );
		}
		else return xo::error_message( "Optimization canceled" );
	}

	result<fitness_t> ModelObjective::EvaluateModel( Model& m, const xo::stop_token& st ) const
	{
		m.SetSimulationEndTime( GetDuration() );
		for ( TimeInSeconds t = evaluation_step_size_; !m.HasSimulationEnded(); t += evaluation_step_size_ )
		{
			if ( st.stop_requested() )
				return xo::error_message( "Optimization canceled" );
			AdvanceSimulationTo( m, t );
		}
		return GetResult( m );
	}

	ModelUP ModelObjective::CreateModelFromParams( Params& par ) const
	{
		auto model = CreateModel( model_factory_props_, par, GetExternalResourceDir() );
		model->SetSimulationEndTime( GetDuration() );

		if ( controller_factory_props_ ) // A controller was defined OUTSIDE the model prop_node
			model->CreateController( controller_factory_props_, par );

		if ( measure_factory_props_ ) // A measure was defined OUTSIDE the model prop_node
			model->CreateMeasure( measure_factory_props_, par );

		return model;
	}

	ModelUP ModelObjective::CreateModelFromParFile( const path& parfile ) const
	{
		SearchPoint params( info_ );
		auto result = params.import_values( parfile );
		log::debug( "Read ", result.first, " of ", info().dim(), " parameters, skipped ", result.second, " from ", parfile.filename() );

		return CreateModelFromParams( params );
	}

	std::vector<path> ModelObjective::WriteResults( const path& file_base )
	{
		// this does not work because we don't have a model member in Objective
		SCONE_THROW_NOT_IMPLEMENTED;
	}

	ModelObjectiveUP CreateModelObjective( const PropNode& scenario_pn, const path& dir )
	{
		// find objective
		FactoryProps opt_props = FindFactoryProps( GetOptimizerFactory(), scenario_pn, "Optimizer" );
		FactoryProps obj_props = FindFactoryProps( GetObjectiveFactory(), opt_props.props(), "Objective" );

		// create ModelObjective object
		auto mob = dynamic_unique_cast<ModelObjective>( CreateObjective( obj_props, dir ) );

		// read mean / std from init file
		if ( opt_props.props().has_key( "init_file" ) && opt_props.props().get< bool >( "use_init_file", true ) )
		{
			auto init_file = scone::FindFile( opt_props.props().get< path >( "init_file" ) );
			spot::par_import_settings pis;
			pis.import_std = opt_props.props().get< bool >( "use_init_file_std", true );
			auto result = mob->info().import_mean_std( init_file,  pis);
			log::debug( "Imported ", result.first, " of ", mob->dim(), ", skipped ", result.second, " parameters from ", init_file );
		}

		// report unused properties
		LogUnusedProperties( obj_props.props() );

		return mob;
	}
}
