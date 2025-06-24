/*
** Measure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Measure.h"
#include "xo/numerical/constants.h"

namespace scone
{
	Measure::Measure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
		Controller( props, par, const_cast<Model&>( model ), loc ) // model is no longer const in Controller parent class
	{
		INIT_PROP( props, name_, "" );
		INIT_PROP( props, weight, 1.0 );
		INIT_PROP( props, threshold, 0.0 );
		INIT_PROP( props, threshold_transition, 0.0 );
		INIT_PROP( props, result_offset, 0.0 );
		INIT_PROP( props, minimize, true );
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

		Real m = *result_ + GetOffset();
		if ( minimize && GetThreshold() != 0 )
		{
			if ( m < GetThreshold() )
				m = 0;
			else if ( m < GetThreshold() + threshold_transition )
				m = m * ( m - GetThreshold() ) / threshold_transition;
		}
		return GetWeight() * m;
	}

	double Measure::GetCurrentResult( const Model& model )
	{
		SCONE_ERROR( "GetCurrentResult() is not implemented for " + GetName() );
	}

	double Measure::GetCurrentWeightedResult( const Model& model )
	{
		Real m = GetCurrentResult( model ) + GetOffset();
		if ( GetThreshold() != 0 )
		{
			if ( m < GetThreshold() )
				m = 0;
			else if ( m < GetThreshold() + threshold_transition )
				m = m * ( m - GetThreshold() ) / threshold_transition;
		}
		return GetWeight() * m;
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

}
