/*
** version.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "version.h"
#include "xo/filesystem/path.h"
#include "xo/filesystem/filesystem.h"
#include "system_tools.h"
#include "xo/string/string_cast.h"
#include "platform.h"
#include <fstream>

#if SCONE_EXPERIMENTAL_FEATURES_ENABLED
	constexpr int SCONE_VERSION_MAJOR = 2;
	constexpr int SCONE_VERSION_MINOR = 1;
	constexpr int SCONE_VERSION_PATCH = 0;
	constexpr const char* SCONE_VERSION_POSTFIX = "BETA 3";
#else
	constexpr int SCONE_VERSION_MAJOR = 2;
	constexpr int SCONE_VERSION_MINOR = 0;
	constexpr int SCONE_VERSION_PATCH = 6;
	constexpr const char* SCONE_VERSION_POSTFIX = "";
#endif

namespace scone
{
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
		static version scone_version = version( SCONE_VERSION_MAJOR, SCONE_VERSION_MINOR, SCONE_VERSION_PATCH, GetSconeBuildNumber(), SCONE_VERSION_POSTFIX );
		return scone_version;
	}
}
