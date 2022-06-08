#pragma once

#include "scone/core/types.h"
#include "xo/filesystem/path.h"
#include "scone/core/PropNode.h"
#include "xo/container/prop_node_tools.h"

namespace scone
{
	/// Settings for input files, used in Optimizer init_files
	struct ParInitSettings
	{
		ParInitSettings() = default;
		ParInitSettings( const PropNode& pn ) : 
			INIT_MEMBER_REQUIRED( pn, file ),
			INIT_MEMBER( pn, use_std, true ),
			INIT_MEMBER( pn, std_factor, 1.0 ),
			INIT_MEMBER( pn, std_offset, 0.0 ),
			INIT_MEMBER( pn, include, "" ),
			INIT_MEMBER( pn, exclude, "" ),
			INIT_MEMBER( pn, locked, false )
		{}

		path file;
		bool use_std = true;
		double std_factor = 1.0;
		double std_offset = 0.0;
		String include;
		String exclude;
		bool locked = false;
	};
}

XO_DEFINE_FROM_PROP_NODE_FOR_TYPE( scone::ParInitSettings );