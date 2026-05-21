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

namespace scone
{
	FeedForwardController::FeedForwardController( const PropNode& props, Params& par, Model& model, const Location& target_area ) :
		Controller( props, par, model, target_area ),
		INIT_MEMBER( props, symmetric, target_area.symmetric_ ),
		INIT_MEMBER( props, include, "*" ),
		INIT_MEMBER( props, exclude, "" )
	{
		INIT_PROP( props, symmetric, target_area.symmetric_ );

		// setup actuator info
		auto incl = xo::pattern_matcher( include );
		auto excl = xo::pattern_matcher( exclude );
		auto& actuators = model.GetActuators();
		for ( size_t idx = 0; idx < actuators.size(); ++idx )
		{
			const auto& name = actuators[idx]->GetName();
			if ( incl( name ) && !excl( name ) )
			{
				ActInfo ai;
				ai.full_name = actuators[idx]->GetName();
				ai.name = GetNameNoSide( ai.full_name );
				ai.side = GetSideFromName( ai.full_name );
				ai.actuator_idx = idx;

				// see if this actuator is on the right side
				if ( target_area.side_ == Side::None || target_area.side_ == ai.side )
					act_infos_.push_back( ai );
			}
		}

		if ( act_infos_.empty() )
			SCONE_ERROR( "No matching actuators (include=\"" + include + "\", exclude=\"" + exclude + "\")" );

		for ( ActInfo& ai : act_infos_ )
		{
			if ( symmetric )
			{
				// check if we've already processed a mirrored version of this ActInfo
				auto it = std::find_if( act_infos_.begin(), act_infos_.end(), [&]( ActInfo& oai ) { return ai.name == oai.name; } );
				if ( it->function_idx != NoIndex )
				{
					ai.function_idx = it->function_idx;
					continue;
				}
			}

			// create a new function
			String prefix = symmetric ? ai.name : ai.full_name;
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
		auto& actuators = model.GetActuators();
		for ( ActInfo& ai : act_infos_ )
		{
			// apply results directly to control value
			actuators[ai.actuator_idx]->AddInput( function_results_[ai.function_idx] );
		}

		return false;
	}

	String FeedForwardController::GetClassSignature() const
	{
		if ( !functions_.empty() )
			return "F" + functions_.front()->GetSignature();
		else return String();
	}
}
