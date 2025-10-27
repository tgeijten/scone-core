/*
** system_tools.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "platform.h"
#include "xo/filesystem/path.h"

namespace scone
{
	using xo::path;

	enum class SconeFolder { Root, Results, Model, Scenarios, Geometry };
	SCONE_API path GetSettingsFolder();
	SCONE_API const path& GetInstallFolder();
	SCONE_API path GetDefaultDataFolder();
	SCONE_API path GetScenarioFolder();
	SCONE_API path GetResultsFolder();
	SCONE_API path GetApplicationFolder();
	SCONE_API path GetFolder( SconeFolder folder );
	SCONE_API path FindFile( const path& filename );
	SCONE_API bool CopyFileLogErrors( const path& source, const path& target, bool overwrite );
}
