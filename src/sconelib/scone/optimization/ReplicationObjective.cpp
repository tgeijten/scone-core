/*
** ReplicationObjective.cpp
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
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

namespace scone
{
	ReplicationObjective::ReplicationObjective( const PropNode& pn, const path& find_file_folder ) :
		ModelObjective( pn, find_file_folder )
	{
		file = FindFile( pn.get<path>( "file" ) );

		if ( signature_postfix.empty() )
			signature_postfix = "RPL";

		// prepare data
		ReadStorageSto( m_Storage, file );
		model_->AddExternalResource( file );

		// make sure data and model are compatible
		auto state = model_->GetState();
		SCONE_THROW_IF( state.GetSize() > m_Storage.GetChannelCount(), "File and model are incompatible for ReplicationObjective" );
		for ( index_t i = 0; i < state.GetSize(); ++i )
			SCONE_THROW_IF( state.GetName( i ) != m_Storage.GetLabels()[ i ], "File and model are incompatible for ReplicationObjective" );

		// find excitation channels
		m_ExcitationChannels.reserve( model_->GetMuscles().size() );
		for ( auto& mus : model_->GetMuscles() )
		{
			m_ExcitationChannels.push_back( m_Storage.TryGetChannelIndex( mus->GetName() + ".excitation" ) );
			SCONE_THROW_IF( m_ExcitationChannels.back() == NoIndex, "Could not find excitation for " + mus->GetName() );
		}

		AddExternalResources( *model_ );
	}

	ReplicationObjective::~ReplicationObjective()
	{}

	void ReplicationObjective::AdvanceSimulationTo( Model& model, TimeInSeconds t ) const
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
		for ( index_t idx = 0; idx < m_Storage.GetFrameCount() && m_Storage.GetFrame( idx ).GetTime() <= t; idx++ )
		{
			auto f = m_Storage.GetFrame( idx );

			// set state and compare output
			model.SetStateValues( f.GetValues(), f.GetTime() );
			for ( index_t idx = 0; idx < m_ExcitationChannels.size(); ++idx )
				result += abs( model.GetMuscles()[ idx ]->GetExcitation() - f[ m_ExcitationChannels[ idx ] ] );
			++samples;
		}

		if ( samples > 0 )
		{
			result = 100 * result / m_ExcitationChannels.size();
			//log::trace( "t=", t, " frames=", frame_count, " start=", frame_start, " result=", result );
			model.GetUserData()[ "RPL_RES" ] = result + model.GetUserData().get< double >( "RPL_RES" );
			model.GetUserData()[ "RPL_SMP" ] = samples + model.GetUserData().get< size_t >( "RPL_SMP" );
		}
	}

	TimeInSeconds ReplicationObjective::GetDuration() const
	{
		return m_Storage.Back().GetTime();
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
