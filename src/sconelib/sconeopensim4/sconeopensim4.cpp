#include "sconeopensim4.h"

#include "scone/core/Factories.h"
#include "ModelOpenSim4.h"
#include "xo/filesystem/path.h"
#include "xo/system/log.h"
#include <OpenSim/OpenSim.h>

namespace scone
{
	void RegisterSconeOpenSim4()
	{
		// Set OpenSim log level to prevent superfluous logging
		OpenSim::Logger::setLevel( OpenSim::Logger::Level::Error );

		// Register ModelOpenSim4
		GetModelFactory().register_type< ModelOpenSim4 >( "ModelOpenSim4" );
		xo::log::info( "Successfully initialized ", ModelOpenSim4::GetOpenSimVersionId() );
	}

	void ConvertModelOpenSim4( const xo::path& inputFile, const xo::path& outputFile )
	{
		OpenSim::Model m( inputFile.str() );
		m.print( outputFile.str() );
	}
}
