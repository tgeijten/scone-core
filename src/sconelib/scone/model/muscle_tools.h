#pragma once

#include "scone/core/platform.h"
#include "scone/core/types.h"
#include <utility>
#include <vector>
#include "scone/core/Angle.h"
#include "xo/filesystem/path.h"
#include "scone/core/PropNode.h"
#include "PathElement.h"

namespace scone
{
	SCONE_API void WriteMuscleInfo( Model& model );
	SCONE_API PropNode GetPathInfo( const std::vector<PathElement>& p );
}

