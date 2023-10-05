/*
** version.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "platform.h"
#include "version.h"

#if __has_include( "scone_version.h" )
#	include "scone_version.h"
#else
constexpr int SCONE_VERSION_MAJOR = 2;
constexpr int SCONE_VERSION_MINOR = 2;
constexpr int SCONE_VERSION_PATCH = 0;
constexpr const char* SCONE_VERSION_POSTFIX = "RC 1";
#endif

#if __has_include( "scone/../../repository_revision.h" )
#	include "scone/../../repository_revision.h"
#endif

#if defined( SCONE_REPOSITORY_REVISION )
constexpr int SCONE_VERSION_BUILD = SCONE_REPOSITORY_REVISION;
#else
constexpr int SCONE_VERSION_BUILD = 0;
#endif

namespace scone
{
	const version& GetSconeVersion()
	{
		static version scone_version = version(
			SCONE_VERSION_MAJOR, SCONE_VERSION_MINOR, SCONE_VERSION_PATCH, SCONE_VERSION_BUILD, SCONE_VERSION_POSTFIX );
		return scone_version;
	}
}
