/*
** Settings.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Settings.h"
#include <mutex>
#include <atomic>
#include "xo/filesystem/filesystem.h"
#include "xo/serialization/serialize.h"
#include "xo/serialization/prop_node_serializer_zml.h"
#include "system_tools.h"
#include "Log.h"
#include "version.h"
#include "Exception.h"
#include "scone_settings_schema.h"

namespace scone
{
	xo::settings LoadSconeSettings()
	{
		auto settings_path = GetSettingsFolder() / "scone-settings.zml";
		xo::settings scone_settings( xo::parse_zml( scone_settings_schema ), settings_path, GetSconeVersion() );
		log::debug( "Loaded settings from ", settings_path );

		return scone_settings;
	}

	xo::settings& GetSconeSettings()
	{
		static xo::settings scone_settings = LoadSconeSettings();
		return scone_settings;
	}
}
