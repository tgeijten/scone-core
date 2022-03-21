/*
** ReplicationObjective.h
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
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

		virtual void AdvanceSimulationTo( Model& m, TimeInSeconds t ) const override;
		virtual TimeInSeconds GetDuration() const override;
		virtual fitness_t GetResult( Model& m ) const override;
		virtual PropNode GetReport( Model& m ) const override;

	private:
		Storage<> m_Storage;
		std::vector< index_t > m_ExcitationChannels;
	};
}
