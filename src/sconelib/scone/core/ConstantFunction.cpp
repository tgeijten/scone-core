/*
** ConstantFunction.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "ConstantFunction.h"
#include "scone/core/string_tools.h"
#include "Exception.h"

namespace scone
{
	ConstantFunction::ConstantFunction( const PropNode& props, Params& par )
	{
		SCONE_ERROR_IF( props.size() != 1, "ConstantController must have one value child" );
		const auto& [name, par_pn] = props.front();
		value = par.get( name, par_pn );
	}

	String ConstantFunction::GetSignature()
	{
		return stringf( "C" );
	}
}
