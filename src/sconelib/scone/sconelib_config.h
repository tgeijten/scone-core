#pragma once

#include "scone/core/Factories.h"

#ifdef SCONE_OPENSIM_3
#include "sconeopensim3/ModelOpenSim3.h"
#endif

#ifdef SCONE_OPENSIM_4
#include "sconeopensim4/ModelOpenSim4.h"
#endif

namespace scone
{
	void RegisterModels()
	{
#ifdef SCONE_OPENSIM_3
	GetModelFactory().register_type< scone::ModelOpenSim3 >( "Simbody" );
	GetModelFactory().register_type< scone::ModelOpenSim3 >( "OpenSim3Model" );
	GetModelFactory().register_type< scone::ModelOpenSim3 >( "OpenSimModel" );
#endif
#ifdef SCONE_OPENSIM_4
	GetModelFactory().register_type< scone::ModelOpenSim34 >( "OpenSim4" );
#endif
	}
}
