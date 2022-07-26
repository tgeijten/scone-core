#include "sconeuser.h"

#include "scone/core/Factories.h"
#include "xo/system/log.h"
#include "ExampleController.h"

namespace scone
{
	SCONE_USER_API void RegisterSconeUserExtensions()
	{
		// register Controllers and Measures here

		// Uncomment the following line to enable ExampleController:
		// GetControllerFactory().register_type< ExampleController >();
	}
}
