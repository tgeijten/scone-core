/*
** Objective.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Objective.h"
#include "scone/core/Exception.h"

namespace scone
{
	Objective::Objective( const PropNode& props, const path& find_file_folder ) :
		HasSignature( props ),
		external_resource_dir_( find_file_folder )
	{
		if ( auto p = props.try_get_child( "Parameters" ) )
		{
			for ( auto& par : *p )
				info().add( ParInfo( par.first, par.second, info().options() ) );
		}

		props.try_get( info().options().auto_std_factor, "auto_std_factor" );
		props.try_get( info().options().auto_std_minimum, "auto_std_minimum" );
	}

	Objective::~Objective()
	{}
}
