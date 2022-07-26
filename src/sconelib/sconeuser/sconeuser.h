#pragma once

#if defined(_MSC_VER)
#	ifdef SCONE_USER_EXPORTS
#		define SCONE_USER_API __declspec(dllexport)
#	else
#		define SCONE_USER_API __declspec(dllimport)
#	endif
#else
#	define SCONE_USER_API
#endif

namespace scone
{
	SCONE_USER_API void RegisterSconeUserExtensions();
}
