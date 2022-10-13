/*
** system_tools.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "system_tools.h"

#include "xo/system/system_tools.h"
#include "xo/filesystem/path.h"
#include "xo/container/prop_node_tools.h"
#include "xo/xo_types.h"
#include "xo/filesystem/filesystem.h"

#include "Settings.h"
#include "Log.h"
#include "Exception.h"

namespace scone
{
	path FindInstallFolder()
	{
		path p = xo::get_application_dir();
		path install_folder;
		for ( ; install_folder.empty() && !p.empty(); p = p.parent_path() )
		{
			if ( path config = p / ".sconeroot"; xo::exists( config ) )
			{
				// binary files are stored in a CMake build folder
				install_folder = xo::load_string( config );
			}
			else if ( xo::exists( p / "scone/scenarios" ) )
			{
				// binary files are part of a scone studio installation
				install_folder = p / "scone";
			}
		}

		if ( install_folder.empty() )
			log::error( "Could not detect SCONE installation folder from ", xo::get_application_dir() );
		else log::debug( "SCONE installation root folder: ", install_folder );

		return install_folder;
	}

	const path& GetInstallFolder()
	{
		static const path g_RootFolder = FindInstallFolder();
		return g_RootFolder;
	}

	path GetFolder( const String& folder, path fallback_path )
	{
		auto p = GetSconeSettings().get< path >( "folders." + folder );
		return !p.empty() ? p : fallback_path;
	}

	path GetSettingsFolder()
	{
		return xo::get_config_dir() / "SCONE";
	}

	path GetDataFolder()
	{
		auto doc = xo::get_documents_dir();
		if ( doc.empty() ) {
			log::warning( "Could not get SCONE data folder, using /SCONE instead" );
			return "/SCONE";
		}
		else return doc / "SCONE";
	}

	path GetApplicationFolder()
	{
		return xo::get_application_dir();
	}

	path GetFolder( SconeFolder folder )
	{
		switch ( folder )
		{
		case SconeFolder::Root: return GetInstallFolder();
		case SconeFolder::Results: return GetFolder( "results", GetDataFolder() / "results" );
		case SconeFolder::Scenarios: return GetFolder( "scenarios", GetDataFolder() );
		case SconeFolder::Geometry: return GetFolder( "geometry", GetInstallFolder().parent_path() / "resources/geometry" );
		default: SCONE_THROW( "Unknown folder type" );
		}
	}

	path FindFile( const path& p )
	{
		if ( xo::current_find_file_path().empty() )
			log::debug( "No find file path set, using current path" );
		
		return xo::find_file( {
			p,
			p.filename(), // filename without rel path
			path( ".." ) / p.filename() // filename in parent folder
			} );
	}
}
