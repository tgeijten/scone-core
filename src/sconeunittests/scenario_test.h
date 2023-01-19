#pragma once

#include "xo/filesystem/path.h"
#include "xo/string/pattern_matcher.h"

namespace scone
{
	void add_scenario_tests( const xo::path& dir,
		const xo::pattern_matcher& include = "*.scone;*.par",
		const xo::pattern_matcher& exclude = "data",
		int recurse_subdir_levels = 1 );
}
