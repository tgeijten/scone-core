/*
** opt_tools.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/core/platform.h"
#include "scone/core/PropNode.h"
#include "scone/core/string_tools.h"
#include "scone/core/memory_tools.h"
#include "scone/core/Log.h"
#include "scone/core/types.h"
#include "scone/optimization/Optimizer.h"
#include "spot/evaluator.h"
#include "spot/reporter.h"
#include "spot/file_reporter.h"

namespace scone
{
	// Log unused properties
	SCONE_API bool LogUnusedProperties( const PropNode& pn );

	// Creates and evaluates SimulationObjective. Logs unused properties.
	SCONE_API PropNode EvaluateScenario( const PropNode& scenario_pn, const path& par_file, const path& output_base );

	// Returns .scone file for a given .par file, or returns argument if already .scone.
	SCONE_API path FindScenario( const path& scenario_or_par_file );

	// Find the model PropNode in a scenario
	SCONE_API PropNode* TryGetModelPropNode( PropNode& scenario_pn );

	// Insert empty "scone_version" and "simulator_version" for when loading older results that don't have versions stored
	SCONE_API void AddEmptyVersionForOldScenarios( PropNode& scenario_pn );

	// Loads a scenario prop_node form a .scone or .par file. Adds empty version for .par files
	SCONE_API PropNode LoadScenario( const path& scenario_or_par_file );

	// Gets spot::evaluator based on SCONE settings
	SCONE_API spot::evaluator& GetSpotEvaluator();

	// Make spot::reporter for internal reporting
	SCONE_API std::unique_ptr<spot::reporter> MakeSpotReporter( Optimizer::OutputMode m );

	// Make spot::file_reporter based on scone settings
	SCONE_API std::unique_ptr<spot::file_reporter> MakeSpotFileReporter( const Optimizer& opt );
}
