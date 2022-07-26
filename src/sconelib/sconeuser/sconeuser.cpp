#include "sconeuser.h"

#include "scone/core/Factories.h"
#include "xo/system/log.h"

namespace scone
{
	SCONE_USER_API void RegisterSconeUser()
	{
		// register custom Controllers and Measures here
		
		// GetControllerFactory().register_type< UserController >();
		// GetMeasureFactory().register_type< UserController >();
	}
}
