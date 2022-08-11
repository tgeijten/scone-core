/*
** version.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "platform.h"
#include "version.h"
#include "system_tools.h"
#include "Log.h"
#include "xo/filesystem/path.h"
#include "xo/filesystem/filesystem.h"
#include "xo/string/string_cast.h"

#if SCONE_EXPERIMENTAL_FEATURES_ENABLED
	constexpr int SCONE_VERSION_MAJOR = 2;
	constexpr int SCONE_VERSION_MINOR = 1;
	constexpr int SCONE_VERSION_PATCH = 0;
	constexpr const char* SCONE_VERSION_POSTFIX = "BETA 4";
#else
	constexpr int SCONE_VERSION_MAJOR = 2;
	constexpr int SCONE_VERSION_MINOR = 0;
	constexpr int SCONE_VERSION_PATCH = 6;
	constexpr const char* SCONE_VERSION_POSTFIX = "";
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
	// DEPRECATED: read build number from .version file
	int GetSconeBuildNumber()
	{
		xo::path versionpath( GetInstallFolder() / ".version" );
		int build = 0;
		if ( xo::exists( versionpath ) )
			xo::from_str( xo::load_string( versionpath ), build );
		return build;
	}

	const version& GetSconeVersion()
	{
		static version scone_version = version( SCONE_VERSION_MAJOR, SCONE_VERSION_MINOR, SCONE_VERSION_PATCH, SCONE_VERSION_BUILD, SCONE_VERSION_POSTFIX );
		return scone_version;
	}
}
