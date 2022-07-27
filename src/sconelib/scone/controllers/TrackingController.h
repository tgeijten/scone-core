/*
** TrackingController.h
** Author(s): Alex Zhou
*/

#pragma once

#include "scone/core/types.h"
#include "scone/controllers/Controller.h"
#include "scone/core/PropNode.h"
#include "scone/optimization/Params.h"
#include "scone/core/Function.h"
#include "scone/model/Leg.h"


namespace scone
{
    /// Controller that produces a feedback control signal for any actuator, based on a pid tracking of a model state.
    /** Example:
    \verbatim
		TrackingController{
			include = "actuator_name"
			pid = [160000., 0.0, 1600.0]
			scale = 0.5
			min_control_value = -1000.0
			max_control_value = 1000.0
			file = "data/tracking_states.sto"
			include_states = "state_name"
		}
    \endverbatim
    */
	class TrackingController : public Controller
	{
	public:
		TrackingController( const PropNode& props, Params& par, Model& model, const Location& target_area );
		virtual ~TrackingController() { };

		/// Bool indicating if function should be the same for left and right; default = true.
		bool symmetric;

		/// Actuator names to include (semicolon separated); default = "*"
		String include;

		/// Actuator names to exclude (semicolon separated); default = ""
		String exclude;

		/// Minimum output for this controller; default = -infinity.
		Real min_control_value;

		/// Maximum output for this controller; default = +infinity.
		Real max_control_value;

		/// Scale value to scale the tracking data
		Real scale;

		virtual bool ComputeControls( Model& model, double timestamp ) override;
		virtual String GetClassSignature() const override;

		/// Filename of storage (sto).
		xo::path file;

		/// States to include for comparison; default = *.
		xo::pattern_matcher include_states;

		/// States to exclude for comparison; default = "".
		xo::pattern_matcher exclude_states;

		/// Time in the .sto file to start measuring; default = 0.
		TimeInSeconds time_offset;

		/// PID controller parameters in the order of position, integral, and derivative
		Vec3 pid;

	private:

		// actuator info
		struct ActInfo
		{
			ActInfo() : side( Side::None ), actuator_idx( NoIndex ) {};
			String name;
			Side side;
			String full_name;
			size_t actuator_idx;
		};

		std::vector< ActInfo > m_ActInfos;

		///Dof& m_SourceDof;

		const Storage<>& storage_;
		struct Channel {
			index_t state_idx_;
			index_t storage_idx_;
			double weight_ = 1.0; //default to 1.0 for now
		};
		std::vector< Channel > state_storage_map_;
		std::vector< std::pair< String, double > > channel_errors_;//current channel error
		std::vector< std::pair< String, double > > channel_errors_last_;//last channel error
		std::vector< std::pair< String, double > > channel_errors_sum_;//sum of channel error

	};
}
