#pragma once

#include "scone/core/types.h"
#include "scone/core/PropNode.h"

namespace scone
{
	/// Settings for muscle activation.
	/** Multiple init statements can be used, for example:
	\verbatim
	Model {
		muscle_activation { include = tib_ant* max_activation = 0.01 }
		muscle_activation { include = gastroc* max_activation = 0.5 }

		...
	}
	\endverbatim
	*/
	struct MuscleActivationSettings
	{
		MuscleActivationSettings() = default;
		MuscleActivationSettings( const PropNode& pn ) :
			INIT_MEMBER( pn, include, "*" ),
			INIT_MEMBER( pn, exclude, "" ),
			min_activation( pn.try_get_child( "min_activation" ) ),
			max_activation( pn.try_get_child( "max_activation" ) ),
			INIT_MEMBER( pn, symmetric, true )
		{}

		/// Pattern matching the file parameters to include (semicolon seperated); default = "" (all).
		String include;

		/// Pattern matching the file parameters to exclude (semicolon seperated); default = "" (none).
		String exclude;

		/// Minimum activation for included muscles; default = model default.
		const PropNode* min_activation;

		/// Maximum activation for included muscles; default = model default.
		const PropNode* max_activation;

		/// Use the same parameter name for left and right; default = true.
		bool symmetric;
	};
}
