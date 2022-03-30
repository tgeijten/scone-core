/*
** ReplicationObjective.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "ReplicationObjective.h"

#include "scone/core/Exception.h"
#include "scone/model/Model.h"
#include "scone/core/string_tools.h"
#include "scone/core/system_tools.h"
#include "scone/core/StorageIo.h"
#include "scone/model/Muscle.h"
#include "scone/core/profiler_config.h"

#include "xo/container/prop_node.h"
#include "xo/container/container_tools.h"
#include "scone/core/Log.h"
#include "xo/container/container_algorithms.h"
#include "xo/container/prop_node_tools.h"

#include <functional>

namespace scone
{
	ReplicationObjective::ReplicationObjective( const PropNode& pn, const path& find_file_folder ) :
		ModelObjective( pn, find_file_folder ),
		INIT_MEMBER( pn, start_time, 0.0 ),
		INIT_MEMBER( pn, stop_time, 0.0 )
	{
		file = FindFile( pn.get<path>( "file" ) );

		if ( signature_postfix.empty() )
			signature_postfix = "RPL";

		// prepare data
		ReadStorageSto( storage_, file );
		AddExternalResource( file );

		// map data
		auto& state = model_->GetState();
		int missing_states = 0;
		state_channels_.reserve( state.GetSize() );
		for ( index_t state_idx = 0; state_idx < state.GetSize(); ++state_idx ) {
			state_channels_.push_back( storage_.TryGetChannelIndex( state.GetName( state_idx ) ) );
			SCONE_THROW_IF( state_channels_.back() == NoIndex, "Could not find state " + state.GetName( state_idx ) );
		}

		// find excitation channels
		excitation_channels_.reserve( model_->GetMuscles().size() );
		for ( auto& mus : model_->GetMuscles() )
		{
			excitation_channels_.push_back( storage_.TryGetChannelIndex( mus->GetName() + ".excitation" ) );
			SCONE_THROW_IF( excitation_channels_.back() == NoIndex, "Could not find excitation for " + mus->GetName() );
		}
	}

	ReplicationObjective::~ReplicationObjective()
	{}

	void ReplicationObjective::AdvanceSimulationTo( Model& model, TimeInSeconds end_time ) const
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		const bool store_data = model.GetStoreData() || !model.GetData().IsEmpty();
		const bool first_frame = !model.GetUserData().has_key( "RPL_err" );
		const auto& muscles = model.GetMuscles();
		SCONE_ASSERT( muscles.size() == excitation_channels_.size() );

		// intermediate results are stored in the model
		auto& mud = model.GetUserData();
		if ( first_frame ) {
			mud[ "RPL_err" ] = 0.0;
			mud[ "RPL_smp" ] = 0;
			if ( store_data )
				for ( index_t idx = 0; idx < muscles.size(); ++idx )
					mud[ "RPL_mus" ][ muscles[ idx ]->GetName() + ".error" ] = 0.0;
		}

		// intermediate values
		std::vector<double> state_values( state_channels_.size() );
		std::vector<double> errors( excitation_channels_.size() );
		double total_error = 0.0;
		size_t samples = 0;

		// loop over data
		auto t = model.GetTime() == 0.0 ? 0.0 : model.GetTime() + model.fixed_control_step_size;
		while ( t < end_time && !model.HasSimulationEnded() )
		{
			const auto& f = storage_.GetClosestFrame( t );
			for ( index_t state_idx = 0; state_idx < state_channels_.size(); ++state_idx )
				state_values[ state_idx ] = f.GetValues()[ state_channels_[ state_idx ] ];

			model.AdvancePlayback( state_values, t );

			// compare excitations
			if ( t >= start_time )
			{
				for ( index_t idx = 0; idx < muscles.size(); ++idx )
				{
					const auto& mus = *muscles[ idx ];
					auto error = abs( mus.GetExcitation() - f[ excitation_channels_[ idx ] ] );
					total_error += error;
					if ( store_data )
					{
						model.GetCurrentFrame()[ mus.GetName() + ".excitation_diff" ] = mus.GetExcitation() - f[ excitation_channels_[ idx ] ];
						errors[ idx ] += error;
					}
				}
				++samples;
			}

			t += model.fixed_control_step_size;

			if ( stop_time != 0 && t > stop_time )
				model.RequestTermination();
		}

		if ( samples > 0 )
		{
			total_error = 100 * total_error / muscles.size();
			//log::trace( "t=", t, " frames=", frame_count, " start=", frame_start, " result=", result );
			mud[ "RPL_err" ] = total_error + mud[ "RPL_err" ].get<double>();
			mud[ "RPL_smp" ] = samples + mud[ "RPL_smp" ].get<size_t>();
			if ( store_data )
				for ( index_t idx = 0; idx < muscles.size(); ++idx ) {
					auto& rpl_mus_pn = mud[ "RPL_mus" ][ muscles[ idx ]->GetName() + ".error" ];
					rpl_mus_pn = rpl_mus_pn.get<double>() + 100 * errors[ idx ];
				}
		}
	}

	TimeInSeconds ReplicationObjective::GetDuration() const
	{
		return storage_.Back().GetTime();
	}

	fitness_t ReplicationObjective::GetResult( Model& m ) const
	{
		auto& mud = m.GetUserData();
		return mud.get< fitness_t >( "RPL_err" ) / mud.get< index_t >( "RPL_smp" );
	}

	PropNode ReplicationObjective::GetReport( Model& m ) const
	{
		auto pn = xo::to_prop_node( GetResult( m ) );
		auto& muscles = m.GetMuscles();
		auto& mus_pn = m.GetUserData()[ "RPL_mus" ];
		auto samples = m.GetUserData().get< index_t >( "RPL_smp" );
		auto errors = mus_pn.get<std::vector<double>>();
		auto sidx = xo::sorted_indices( errors, std::greater<double>() );
		for ( auto idx : sidx )
			pn[ muscles[ idx ]->GetName() + ".error" ] = mus_pn[ idx ].get<double>() / samples;
		return pn;
	}
}
