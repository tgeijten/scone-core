#pragma once

#include "scone/core/types.h"
#include "xo/filesystem/path.h"
#include "scone/core/PropNode.h"
#include "xo/container/prop_node_tools.h"

namespace scone
{
	/// Settings for parameter initialization files (.par), used by Optimizer.
	/** Multiple init statements can be used, for example:
	\verbatim
	CmaOptimizer {
		init { file = par_file1.par std_offset = 0.1 }
		init { file = par_file2.par exclude = "*.offset" }

		...
	}
	\endverbatim
	*/
	struct ParInitSettings
	{
		ParInitSettings() = default;
		ParInitSettings( const PropNode& pn ) :
			INIT_MEMBER_REQUIRED( pn, file ),
			INIT_MEMBER( pn, use_std, true ),
			INIT_MEMBER( pn, std_factor, 1.0 ),
			INIT_MEMBER( pn, std_offset, 0.0 ),
			INIT_MEMBER( pn, value_factor, 1.0 ),
			INIT_MEMBER( pn, value_offset, 0.0 ),
			INIT_MEMBER( pn, include, "" ),
			INIT_MEMBER( pn, exclude, "" ),
			INIT_MEMBER( pn, locked, false ),
			INIT_MEMBER( pn, use_best_as_mean, false )
		{}

		/// Parameter file (.par) to be used for initial parameter values.
		path file;

		/// Use the standard deviations from the file; default = 1.
		bool use_std = true;

		/// Factor by which to multiply the standard deviations from the init_file; default = 1.0.
		double std_factor = 1.0;

		/// Offset added to the standard deviations from init_file; default = 0.
		double std_offset = 0.0;

		/// Factor by which to multiply the parameter values; default = 1.0.
		double value_factor = 1.0;

		/// Offset added to the parameter values; default = 0.
		double value_offset = 0.0;

		/// Pattern matching the file parameters to include (semicolon seperated); default = "" (all).
		String include;

		/// Pattern matching the file parameters to exclude (semicolon seperated); default = "" (none).
		String exclude;

		/// Load the 'best' parameters (first value column) from the file and prevent them from be optimized; default = 0.
		bool locked = false;

		/// Use values from the first column of the .par file (generation best) as mean; default = false.
		bool use_best_as_mean = false;
	};

	inline spot::par_import_settings to_spot( const ParInitSettings& p ) {
		spot::par_import_settings pis;
		pis.import_std = p.use_std;
		pis.std_factor = p.std_factor;
		pis.std_offset = p.std_offset;
		pis.value_factor = p.value_factor;
		pis.value_offset = p.value_offset;
		pis.include = p.include;
		pis.exclude = p.exclude;
		pis.locked = p.locked;
		pis.use_best_as_mean = p.use_best_as_mean;
		return pis;
	}
}

XO_DEFINE_FROM_PROP_NODE_FOR_TYPE( scone::ParInitSettings );