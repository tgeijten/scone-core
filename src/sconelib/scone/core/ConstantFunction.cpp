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
	ConstantFunction::ConstantFunction( const PropNode& props, Params& par ) :
		INIT_PAR_MEMBER( props, par, value, 0.0 )
	{}

	String ConstantFunction::GetSignature()
	{
		return stringf( "C" );
	}
}
