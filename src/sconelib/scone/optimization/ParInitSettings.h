#pragma once

#include "scone/core/types.h"
#include "xo/filesystem/path.h"
#include "scone/core/PropNode.h"
#include "xo/container/prop_node_tools.h"

namespace scone
{
	/// Settings for parameter initialization files (.par), used by Optimizer.
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

		/// Parameter file (.par) to be used for initial parameter values.
		path file;

		/// Use the standard deviations from the init_file; default = 1.
		bool use_std = true;

		/// Factor by which to multiply the standard deviations from the init_file; default = 1.0.
		double std_factor = 1.0;

		/// Offset added to the standard deviations from init_file; default = 0.
		double std_offset = 0.0;

		/// Pattern matching the init_file parameters to include (semicolon seperated); default = "" (all).
		String include;

		/// Pattern matching the init_file parameters to exclude (semicolon seperated); default = "" (none).
		String exclude;

		/// Lock the parameters from this file, preventing them to be optimized; default = 0.
		bool locked = false;
	};
}

XO_DEFINE_FROM_PROP_NODE_FOR_TYPE( scone::ParInitSettings );