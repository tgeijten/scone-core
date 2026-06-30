/*
** StepMeasure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "StepMeasure.h"
#include "scone/model/Model.h"
#include "scone/core/Log.h"
#include "scone/core/Exception.h"
#include "scone/core/profiler_config.h"
#include "scone/core/GaitCycle.h"

namespace scone
{
	StepMeasure::StepMeasure( const PropNode& props, Params& par,
		const Model& model, const Location& loc ) :
		Measure( props, par, model, loc ),
		INIT_MEMBER( props, stride_length, RangePenalty<Real>() ),
		INIT_MEMBER( props, stride_width, RangePenalty<Real>() ),
		INIT_MEMBER( props, stride_duration, RangePenalty<Real>() ),
		INIT_MEMBER( props, stride_velocity, RangePenalty<Real>() ),
		INIT_MEMBER( props, load_threshold, 0.01 ),
		INIT_MEMBER( props, min_stance_duration_threshold, 0.1 ),
		INIT_MEMBER( props, initiation_cycles, 1 ),
		penalties_{
			{ &stride_length, "stride_length" },
			{ &stride_width, "stride_width" },
			{ &stride_duration, "stride_duration" },
			{ &stride_velocity, "stride_velocity" }
		}
	{
		SCONE_THROW_IF( initiation_cycles < 1, "initiation_cycles should be >= 1" );

		auto pencount = xo::count_if( penalties_, [&]( auto&& p ) { return !p.first->IsNull(); } );
		SCONE_THROW_IF( pencount == 0, "No penalties defined in StepMeasure" );
	}

	UpdateResult StepMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );
		
		// reserve space for better performance
		if ( stored_data_.IsEmpty() ) {
			size_t s = size_t( 0.5 + model.GetSimulationEndTime() / model.fixed_measure_step_size );
			stored_data_.Reserve( s );
			for ( const auto& leg : model.GetLegs() ) {
				stored_data_.AddChannels( leg.GetName() + ".grf_norm", { "_x", "_y", "_z" } );
				stored_data_.AddChannels( leg.GetName() + ".cop", { "_x", "_y", "_z" } );
			}
		}

		auto& frame = stored_data_.AddFrame( timestamp );
		for ( index_t idx = 0; idx < model.GetLegCount(); ++idx )
		{
			const auto& leg = model.GetLeg( idx );
			auto fv = leg.GetContactForceValue();
			Vec3 grf = fv.force / model.GetBW();
			frame.SetVec3( idx * 6, grf );
			frame.SetVec3( idx * 6 + 3, fv.point );
		}

		return false;
	}

	double StepMeasure::ComputeResult( const Model& model )
	{
		GaitCycleExtractionSettings cfg{ load_threshold, min_stance_duration_threshold };
		auto cycles = ExtractGaitCycles( stored_data_, cfg );

		// calculate stride length / duration / velocity
		for ( index_t idx = initiation_cycles; idx < cycles.size(); ++idx )
		{
			if ( !stride_length.IsNull() )
				stride_length.AddSample( cycles[idx].length() );
			if ( !stride_width.IsNull() )
				stride_width.AddSample( cycles[idx].width() );
			if ( !stride_duration.IsNull() )
				stride_duration.AddSample( cycles[idx].duration() );
			if ( !stride_velocity.IsNull() )
				stride_velocity.AddSample( cycles[idx].velocity() );
		}

		// calculate penalty
		Real penalty = 0;
		for ( auto&& [pen, name] : penalties_ ) {
			if ( !pen->IsNull() ) {
				penalty += pen->GetResult();
				report_.set( name + "_penalty", pen->GetResult() );
			}
		}

		return penalty;
	}

	String StepMeasure::GetClassSignature() const
	{
		if ( !stride_length.IsNull() )
			return stringf( "S%d", int( 100 * stride_length.min ) );
		else if ( !stride_duration.IsNull() )
			return stringf( "SD%d", int( 100 * stride_duration.min ) );
		else if ( !stride_velocity.IsNull() )
			return stringf( "SV%d", int( 100 * stride_velocity.min ) );
		return stringf( "S" );
	}
}
