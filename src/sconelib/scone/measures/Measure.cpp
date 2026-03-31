/*
** Measure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Measure.h"
#include "xo/numerical/constants.h"
#include "scone/model/Model.h"

namespace scone
{
	Measure::Measure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
		Controller( props, par, const_cast<Model&>( model ), loc ) // model is no longer const in Controller parent class
	{
		INIT_PROP( props, name_, "" );
		INIT_PROP( props, weight, 1.0 );
		INIT_OPTIONAL_PROP( props, threshold );
		INIT_OPTIONAL_PROP( props, soft_threshold );
		INIT_PROP( props, threshold_transition, 0.0 );
		INIT_PROP( props, result_offset, 0.0 );
		INIT_PROP( props, minimize, true );
		INIT_PROP( props, scale_weight_by_relative_duration, false );
		if ( soft_threshold ) {
			SCONE_ERROR_IF( threshold || result_offset != 0.0, "Cannot set threshold or result_offset with soft_threshold" )
			result_offset = -*soft_threshold;
			threshold = 0;
		}
	}

	double Measure::GetResult( const Model& model )
	{
		if ( !result_ )
			result_ = ComputeResult( model );
		return *result_;
	}

	double Measure::GetWeightedResult( const Model& model )
	{
		if ( !result_ )
			result_ = ComputeResult( model );

		// result_offset is used by soft_threshold
		Real m = *result_ + result_offset;

		// apply threshold
		if ( minimize && threshold )
		{
			if ( m < *threshold )
				m = 0;
			else if ( m < *threshold + threshold_transition )
				m = m * ( m - *threshold ) / threshold_transition;
		}

		// scale by relative duration after threshold
		if ( scale_weight_by_relative_duration )
			m *= GetRelativeDuration( model );

		return weight * m;
	}

	double Measure::GetCurrentResult( const Model& model )
	{
		SCONE_ERROR( "GetCurrentResult() is not implemented for " + GetName() );
	}

	double Measure::GetCurrentWeightedResult( const Model& model )
	{
		Real m = GetCurrentResult( model ) + result_offset;
		if ( threshold )
		{
			if ( m < *threshold )
				m = 0;
			else if ( m < *threshold + threshold_transition )
				m = m * ( m - *threshold ) / threshold_transition;
		}
		return weight * m;
	}

	void Measure::Reset( Model& model )
	{
		result_.reset();
		report_.clear();
	}

	const String& Measure::GetName() const
	{
		if ( name_.empty() )
			name_ = xo::get_clean_type_name( *this );
		return name_;
	}

	UpdateResult Measure::PerformAnalysis( const Model& model, double timestamp )
	{
		// #todo: cleanup, rename UpdateMeasure into PerformAnalysis
		return UpdateMeasure( model, timestamp );
	}

	double Measure::WorstResult() const
	{
		return minimize ? xo::constants<double>::max() : xo::constants<double>::lowest();
	}

	double Measure::GetRelativeDuration( const Model& model ) const
	{
		auto model_end_time = model.GetSimulationEndTime();
		auto max_end_time = stop_time > 0.0 ? std::min( stop_time, model_end_time ) : model_end_time;
		auto max_duration = max_end_time - start_time;
		SCONE_ERROR_IF( max_duration <= 0.0, "Measure start_time must be smaller than stop_time or duration" );
		auto end_time = stop_time > 0.0 ? std::min( stop_time, model.GetTime() ) : model.GetTime();
		auto duration = std::max( 0.0, end_time - start_time );
		return duration / max_duration;
	}
}
