/*
** ReplicationObjective.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/optimization/Objective.h"
#include "scone/core/PropNode.h"
#include "scone/core/Storage.h"

#include "xo/filesystem/path.h"
#include "ModelObjective.h"

#include <vector>
#include <utility>

namespace scone
{
	/// Objective for replicating excitation channels from an existing simulation result, without performing a new a simulation.
	class SCONE_API ReplicationObjective : public ModelObjective
	{
	public:
		ReplicationObjective( const PropNode& props, const path& find_file_folder );
		virtual ~ReplicationObjective();

		/// File containing the existing simulation results (.sto).
		path file;

		TimeInSeconds start_time;
		TimeInSeconds stop_time;
		xo::pattern_matcher include_, exclude_;
		bool use_squared_error;
		bool use_muscle_activation;
		Real muscle_activation_rate_;
		Real muscle_deactivation_rate_;
		Real fixed_control_step_size;

		virtual void AdvanceSimulationTo( Model& m, TimeInSeconds t ) const override;
		virtual TimeInSeconds GetDuration() const override;
		virtual fitness_t GetResult( Model& m ) const override;
		virtual PropNode GetReport( Model& m ) const override;

	private:
		Storage<> storage_;
		std::vector<index_t> state_channels_;
		xo::flat_map< index_t, index_t > muscle_excitation_map_;
		std::vector<std::vector<Real>> state_storage_;
		std::vector<index_t> storage_indices_;
	};
}
