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
#include <charconv>

namespace scone
{
	inline void set_exact( PropNode& pn, const double& value ) {
		char buf[ 40 ];
		*std::to_chars( buf, buf + sizeof( buf ) - 1, value ).ptr = '\0';
		pn.set_value( buf );
	}
	inline double get_exact( PropNode& pn ) {
		double value;
		const string& str = pn.get_str();
		std::from_chars( str.c_str(), str.c_str() + str.size(), value );
		return value;
	}

	ReplicationObjective::ReplicationObjective( const PropNode& pn, const path& find_file_folder ) :
		ModelObjective( pn, find_file_folder ),
		INIT_MEMBER( pn, start_time, 0.0 ),
		INIT_MEMBER( pn, stop_time, 0.0 ),
		INIT_MEMBER( pn, include_, "*" ),
		INIT_MEMBER( pn, exclude_, "" ),
		INIT_MEMBER( pn, use_squared_error, false ),
		INIT_MEMBER( pn, use_muscle_activation, false ),
		INIT_MEMBER( pn, muscle_activation_rate_, 100.0 ),
		INIT_MEMBER( pn, muscle_deactivation_rate_, 25.0 )
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
		auto& muscles = model_->GetMuscles();
		muscle_excitation_map_.reserve( muscles.size() );
		for ( index_t midx = 0; midx < muscles.size(); ++midx )
		{
			const auto& name = muscles[ midx ]->GetName();
			if ( include_( name ) && !exclude_( name ) ) {
				auto cidx = storage_.TryGetChannelIndex( name + ".excitation" );
				if ( cidx != NoIndex )
					muscle_excitation_map_[ midx ] = cidx;
				else log::warning( "Could not find excitation channel for ", name, " in ", file );
			}
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

		// intermediate results are stored in the model
		auto& mud = model.GetUserData();
		if ( first_frame ) {
			mud[ "RPL_err" ] = 0;
			mud[ "RPL_smp" ] = 0;
			if ( store_data )
				for ( index_t idx = 0; idx < muscles.size(); ++idx )
					mud[ "RPL_mus" ][ muscles[ idx ]->GetName() + ".error" ] = 0;
			mud[ "RPL_act" ].add_children( muscles.size(), "", "0" );
		}

		// intermediate values
		std::vector<double> state_values( state_channels_.size() );
		std::vector<double> errors( muscles.size(), 0.0 );
		double total_error = 0.0;
		size_t samples = 0;

		// initialize activations
		auto& act_pn = mud[ "RPL_act" ];
		std::vector<double> activations( muscles.size() );
		for ( index_t idx = 0; idx < muscles.size(); ++idx )
			activations[ idx ] = get_exact( act_pn[ idx ] );

		// loop over data
		auto t = model.GetTime() == 0.0 ? 0.0 : model.GetTime() + model.fixed_control_step_size;
		while ( t < end_time && !model.HasSimulationEnded() )
		{
			const auto dt = model.fixed_control_step_size;

			const auto& f = storage_.GetClosestFrame( t );
			for ( index_t state_idx = 0; state_idx < state_channels_.size(); ++state_idx )
				state_values[ state_idx ] = f.GetValues()[ state_channels_[ state_idx ] ];

			model.AdvancePlayback( state_values, t );

			// update activations based on computed excitations
			auto a0 = activations[ 0 ];
			if ( t > 0.0 )
			{
				double cd = muscle_deactivation_rate_;
				double c1 = muscle_activation_rate_ - cd;
				for ( index_t idx = 0; idx < muscles.size(); ++idx ) {
					auto u = muscles[ idx ]->GetExcitation();
					activations[ idx ] = u - ( u - activations[ idx ] ) * std::exp( -c1 * u * dt - cd * dt );
				}
			}
			else for ( index_t idx = 0; idx < muscles.size(); ++idx )
				activations[ idx ] = muscles[ idx ]->GetExcitation();

			// always store new activation and org excitation
			if ( store_data )
			{
				for ( const auto& [midx, cidx] : muscle_excitation_map_ )
				{
					const auto& mus = *muscles[ midx ];
					model.GetCurrentFrame()[ mus.GetName() + ".activation_new" ] = activations[ midx ];
					model.GetCurrentFrame()[ mus.GetName() + ".excitation_org" ] = f[ cidx ];
				}
			}

			// compare excitations
			if ( t >= start_time )
			{
				for ( const auto& [midx, cidx] : muscle_excitation_map_ )
				{
					const auto& mus = *muscles[ midx ];
					auto excitation_diff = mus.GetExcitation() - f[ cidx ];
					auto activation_diff = activations[ midx ] - mus.GetActivation();
					auto diff = use_muscle_activation ? activation_diff : excitation_diff;
					auto error = use_squared_error ? xo::squared( diff ) : std::abs( diff );
					total_error += error;
					if ( store_data )
					{
						model.GetCurrentFrame()[ mus.GetName() + ".excitation_diff" ] = excitation_diff;
						model.GetCurrentFrame()[ mus.GetName() + ".activation_diff" ] = activation_diff;
						model.GetCurrentFrame()[ mus.GetName() + ".error" ] = error;
						errors[ midx ] += error;
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
			set_exact( mud[ "RPL_err" ], get_exact( mud[ "RPL_err" ] ) + total_error );
			set_exact( mud[ "RPL_smp" ], get_exact( mud[ "RPL_smp" ] ) + samples );
			if ( store_data )
				for ( index_t idx = 0; idx < muscles.size(); ++idx ) {
					auto& rpl_mus_pn = mud[ "RPL_mus" ][ muscles[ idx ]->GetName() + ".error" ];
					set_exact( rpl_mus_pn, get_exact( rpl_mus_pn ) + 100 * errors[ idx ] );
				}
		}

		if ( t > 0 && !model.HasSimulationEnded() ) {
			// keep activations for next round
			for ( index_t idx = 0; idx < muscles.size(); ++idx )
				set_exact( act_pn[ idx ], activations[ idx ] );
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