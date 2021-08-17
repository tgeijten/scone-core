#include "sconeopensim3.h"

#include "scone/core/Factories.h"
#include "ModelOpenSim3.h"
#include "xo/system/log.h"

namespace scone
{
	SCONE_OPENSIM_3_API void RegisterSconeOpenSim3()
	{
		GetModelFactory().register_type< ModelOpenSim3 >( "Simbody" ); // backwards compatibility
		GetModelFactory().register_type< ModelOpenSim3 >( "OpenSim3Model" );
		GetModelFactory().register_type< ModelOpenSim3 >( "OpenSimModel" );
		GetModelFactory().register_type< ModelOpenSim3 >( "ModelOpenSim3" );
		xo::log::info( "Successfully initialized OpenSim3 version ", ModelOpenSim3::GetOpenSimBuildVersion() );
	}
}
