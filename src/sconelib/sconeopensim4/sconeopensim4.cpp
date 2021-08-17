#include "sconeopensim4.h"

#include "scone/core/Factories.h"
#include "ModelOpenSim4.h"
#include "xo/filesystem/path.h"
#include "xo/system/log.h"

namespace scone
{
	void RegisterSconeOpenSim4()
	{
		GetModelFactory().register_type< ModelOpenSim4 >( "ModelOpenSim4" );
		xo::log::info( "Successfully initialized OpenSim4 version ", ModelOpenSim4::GetOpenSimBuildVersion() );
	}

	void ConvertModelOpenSim4( const xo::path& inputFile, const xo::path& outputFile )
	{
		OpenSim::Model m( inputFile.str() );
		m.print( outputFile.str() );
	}
}
