/*
** MimicMeasure.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Measure.h"
#include "scone/core/Statistic.h"
#include "xo/string/pattern_matcher.h"

namespace scone
{
	/// Measure for how well a simulation mimics data from predefined motion file.
	/** The measure computes the mean squared error (MSE) between the simulated state and the
	states in a .sto file. Only variables are considered that are part of the model state,
	which includes DOFs, DOF velocities, muscle activation (*.activation), and muscle fiber length (*.fiber_length)
	Variables that are derived from the state are not included in the measure.

	When using a Hyfydy model with OpenSim state data, make sure to add ''include_dofs_in_state = 1'' to ModelHfd.
	
	Example:
	\verbatim
	# Measure for mimicking 2D gait
	MimicMeasure {
		name = Mimic
		weight = 10000
		file = my_data_file.sto
		include_states = "*.pos.x;*.pos.y;*.ori.z;*.activation"
		stop_time = 2
		threshold = 0.01
		average_error_limit = 0.01
		activation_error_weight = 0.25
	}
	\endverbatim
	*/
	class MimicMeasure : public Measure
	{
	public:
		MimicMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc );

		/// Filename of storage (sto).
		xo::path file;

		/// States to include for comparison; default = *.
		xo::pattern_matcher include_states;

		/// States to exclude for comparison; default = "".
		xo::pattern_matcher exclude_states;

		/// Use only best match instead of average match -- useful when data contains a single pose; default = false.
		bool use_best_match;

		/// Average error above which to terminate simulation early (if set to non-zero); default = 0.
		Real average_error_limit;

		/// Peak error above which to terminate simulation early (if set to non-zero); default = ''2 * average_error_limit''.
		Real peak_error_limit;

		/// Time in the .sto file to start measuring; default = 0.
		TimeInSeconds time_offset;

		/// weight applied to muscle activation error; default = 1.
		Real activation_error_weight;

		virtual bool UpdateMeasure( const Model& model, double timestamp ) override;
		virtual double ComputeResult( const Model& model ) override;
		virtual double GetCurrentResult( const Model& model ) override;
		virtual void Reset( Model& model ) override;
		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;

	protected:
		virtual String GetClassSignature() const override;
		const Storage<>& storage_;
		Statistic<> mimic_result_;
		struct Channel {
			index_t state_idx_;
			index_t storage_idx_;
			double weight_ = 1.0;
		};
		std::vector< Channel > state_storage_map_;
		std::vector< std::pair< String, double > > channel_errors_;

		TimeInSeconds termination_time_;
	};
}
