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
		Measure( props, par, model, loc )
	{
		INIT_PROP( props, stride_length, RangePenalty<Real>() );
		INIT_PROP( props, stride_duration, RangePenalty<Real>() );
		INIT_PROP( props, stride_velocity, RangePenalty<Real>() );
		INIT_PROP( props, load_threshold, 0.01 );
		INIT_PROP( props, min_stance_duration_threshold, 0.2 );
		INIT_PROP( props, initiation_cycles, 1 );

		SCONE_THROW_IF( initiation_cycles < 1, "initiation_cycles should be >= 1" );
		SCONE_THROW_IF( stride_length.IsNull() && stride_duration.IsNull() && stride_velocity.IsNull(),
			"Any of stride_length / stride_duration / stride_velocity should be defined" );
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
			if ( !stride_duration.IsNull() )
				stride_duration.AddSample( cycles[idx].duration() );
			if ( !stride_velocity.IsNull() )
				stride_velocity.AddSample( cycles[idx].velocity() );
		}

		// calculate penalty
		double penalty = 0;
		if ( !stride_length.IsNull() )
		{
			penalty += stride_length.GetResult();
			report_.set( "stride_length_penalty", stride_length.GetResult() );
		}
		if ( !stride_duration.IsNull() )
		{
			penalty += stride_duration.GetResult();
			report_.set( "stride_duration_penalty", stride_duration.GetResult() );
		}
		if ( !stride_velocity.IsNull() )
		{
			penalty += stride_velocity.GetResult();
			report_.set( "stride_velocity_penalty", stride_velocity.GetResult() );
		}

		return penalty;
	}

	String StepMeasure::GetClassSignature() const
	{
		return stringf( "SL" );
	}
}
