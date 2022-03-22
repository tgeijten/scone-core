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

namespace scone
{
	ReplicationObjective::ReplicationObjective( const PropNode& pn, const path& find_file_folder ) :
		ModelObjective( pn, find_file_folder )
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
			SCONE_THROW_IF( state_channels_.back() == NoIndex , "Could not find state " + state.GetName( state_idx ) );
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

		// intermediate results are stored in the model
		if ( !model.GetUserData().has_key( "RPL_RES" ) )
		{
			model.GetUserData()[ "RPL_RES" ] = 0.0;
			model.GetUserData()[ "RPL_SMP" ] = 0;
		}

		// compute result
		double result = 0.0;
		size_t samples = 0;
		std::vector<double> state_values( state_channels_.size() );
		auto t = model.GetTime() == 0.0 ? 0.0 : model.GetTime() + model.fixed_control_step_size;
		for (; t < end_time; t += model.fixed_control_step_size )
		{
			const auto& f = storage_.GetClosestFrame( t );
			for ( index_t state_idx = 0; state_idx < state_channels_.size(); ++state_idx )
				state_values[ state_idx ] = f.GetValues()[ state_channels_[ state_idx ] ];

			model.AdvancePlayback( state_values, t );

			// compare excitations
			for ( index_t idx = 0; idx < excitation_channels_.size(); ++idx )
				result += abs( model.GetMuscles()[ idx ]->GetExcitation() - f[ excitation_channels_[ idx ] ] );
			++samples;
		}

		if ( samples > 0 )
		{
			result = 100 * result / excitation_channels_.size();
			//log::trace( "t=", t, " frames=", frame_count, " start=", frame_start, " result=", result );
			model.GetUserData()[ "RPL_RES" ] = result + model.GetUserData().get< double >( "RPL_RES" );
			model.GetUserData()[ "RPL_SMP" ] = samples + model.GetUserData().get< size_t >( "RPL_SMP" );
		}
	}

	TimeInSeconds ReplicationObjective::GetDuration() const
	{
		return storage_.Back().GetTime();
	}

	fitness_t ReplicationObjective::GetResult( Model& m ) const
	{
		return m.GetUserData().get< fitness_t >( "RPL_RES" ) / ( m.GetUserData().get< index_t >( "RPL_SMP" ) );
	}

	PropNode ReplicationObjective::GetReport( Model& m ) const
	{
		return xo::to_prop_node( GetResult( m ) );
	}
}
