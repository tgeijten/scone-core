/*
** HasSignature.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "HasSignature.h"
#include "string_tools.h"
#include "xo/string/string_tools.h"
#include "system_tools.h"
#include "version.h"

namespace scone
{
	HasSignature::HasSignature( const PropNode& pn )
	{
		INIT_PROP( pn, signature_prefix, String("") );
		INIT_PROP( pn, signature_postfix, String("") );
		INIT_PROP( pn, signature, String("") );

		// replace DATE_TIME tag with (yes, indeed) DATE and TIME
		ReplaceStringTags( signature_prefix );
		ReplaceStringTags( signature_postfix );
	}

	String HasSignature::GetSignature() const
	{
		return xo::concatenate_str( { signature_prefix, signature.empty() ? GetClassSignature() : signature, signature_postfix }, "." );
	}
}
