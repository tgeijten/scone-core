/*
** Controller.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Controller.h"
#include "spot/par_tools.h"
#include "scone/core/Log.h"

namespace scone
{
	Controller::Controller( const PropNode& props, Params& par, Model& model, const Location& target_area ) :
		HasSignature( props ),
		INIT_MEMBER( props, name_, "" ),
		INIT_PAR_MEMBER( props, par, start_time, 0.0 ),
		INIT_PAR_MEMBER( props, par, stop_time, 0.0 ),
		INIT_MEMBER( props, disabled_, false )
	{
		// add custom parameters
		if ( auto par_pn = props.try_get_child( "Parameters" ) )
		{
			for ( auto& [key, value] : *par_pn )
				par.get( key, value );
		}
	}

	Controller::~Controller()
	{}

	bool Controller::UpdateControls( Model& model, double timestamp )
	{
		if ( IsActive( model, timestamp ) )
			return ComputeControls( model, timestamp );
		else return false;
	}

	bool Controller::UpdateAnalysis( const Model& model, double timestamp )
	{
		if ( IsActive( model, timestamp ) )
			return PerformAnalysis( model, timestamp );
		else return false;
	}

	void Controller::Reset( Model& model )
	{
		log::warning( "Reset() is not implemented for ", name_ );
	}
}
