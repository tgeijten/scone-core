#include "profiler_config.h"

namespace scone
{
	static bool g_profiler_enabled = false;

	bool SetProfilerEnabled( bool enabled )
	{
		auto prev_value = g_profiler_enabled;
		g_profiler_enabled = enabled;
		return prev_value;
	}

	bool GetProfilerEnabled()
	{
		return g_profiler_enabled;
	}
}
