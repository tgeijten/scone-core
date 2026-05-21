/*
** FeedForwardController.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "FeedForwardController.h"

#include "scone/controllers/Controller.h"
#include "scone/model/Muscle.h"
#include "scone/model/Location.h"

#include "scone/core/Factories.h"
#include "scone/core/profiler_config.h"
#include "scone/core/IncludeExcludePattern.h"

namespace scone
{
	FeedForwardController::FeedForwardController( const PropNode& props, Params& par, Model& model, const Location& target_area ) :
		Controller( props, par, model, target_area ),
		INIT_MEMBER( props, symmetric, target_area.symmetric_ ),
		INIT_MEMBER( props, include, "*" ),
		INIT_MEMBER( props, exclude, "" )
	{
		// setup actuator info
		IncludeExcludePattern pat( include, exclude );
		for ( auto* act : model.GetActuators() )
		{
			bool good_side = target_area.side_ == Side::None || target_area.side_ == act->GetSide();
			if ( good_side && pat( act->GetName() ) )
				act_infos_.emplace_back( act );
		}

		if ( act_infos_.empty() )
			SCONE_ERROR( "No matching actuators (include=\"" + include + "\", exclude=\"" + exclude + "\")" );

		// create functions
		for ( auto& ai : act_infos_ )
		{
			auto name = GetNameNoSide( ai.actuator->GetName() );
			if ( symmetric )
			{
				auto it = xo::find_if( act_infos_, [&]( auto& o ) { return xo::str_begins_with( o.actuator->GetName(), name ); } );
				if ( it != act_infos_.end() && it->function_idx != no_index ) {
					ai.function_idx = it->function_idx;
					continue;
				}
			}

			// create a new function
			String prefix = symmetric ? name : ai.actuator->GetName();
			ScopedParamSetPrefixer prefixer( par, prefix + "." );
			auto fp = FindFactoryProps( GetFunctionFactory(), props, "Function" );
			functions_.push_back( CreateFunction( fp, par ) );
			ai.function_idx = functions_.size() - 1;
		}
		function_results_.resize( functions_.size() );
	}

	bool FeedForwardController::ComputeControls( Model& model, double time )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		// evaluate functions
		for ( size_t idx = 0; idx < functions_.size(); ++idx )
			function_results_[idx] = functions_[idx]->GetValue( time );

		// apply results to all actuators
		for ( auto& ai : act_infos_ )
			ai.actuator->AddInput( function_results_[ai.function_idx] );

		return false;
	}

	String FeedForwardController::GetClassSignature() const
	{
		if ( !functions_.empty() )
			return "F" + functions_.front()->GetSignature();
		else return String();
	}
}
