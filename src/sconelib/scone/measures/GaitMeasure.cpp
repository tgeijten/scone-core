/*
** GaitMeasure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "GaitMeasure.h"
#include "scone/model/Model.h"
#include "scone/model/Body.h"
#include "scone/core/Log.h"
#include "scone/model/Muscle.h"
#include "scone/core/profiler_config.h"
#include "scone/core/Range.h"
#include "xo/container/sorted_vector.h"
#include "xo/container/container_algorithms.h"
#include "xo/geometry/dynvec.h"

namespace scone
{
	GaitMeasure::GaitMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
		Measure( props, par, model, loc )
	{
		INIT_PROP( props, termination_height, 0.5 );
		INIT_PROP( props, min_velocity, 0 );
		INIT_PROP( props, max_velocity, 299792458.0 ); // default max velocity = speed of light
		INIT_PROP( props, load_threshold, 0.1 );
		INIT_PROP( props, min_step_duration, 0.1 );
		INIT_PROP( props, initiation_steps, 2 );
		INIT_PROP( props, base_bodies, "" );
		INIT_PROP( props, direction, Vec3::unit_x() );
		INIT_PROP( props, use_initial_heading, false );

		if ( use_initial_heading && model.HasRootBody() )
			direction = xo::projected_xz( model.GetRootBody().GetOrientation() * Vec3::unit_x() );
		xo::normalize( direction );

		if ( !base_bodies.empty() )
		{
			// extract individual body names from gait_bodies string
			auto tokens = xo::split_str( base_bodies, ";, " );
			for ( const String& t : tokens )
				m_BaseBodies.push_back( &( *FindByName( model.GetBodies(), t ) ) );
		}
		else
		{
			// use foot_body for base bodies
			for ( const auto& l : model.GetLegs() )
				m_BaseBodies.push_back( &l.GetFootBody() );
		}

		SCONE_ERROR_IF( m_BaseBodies.size() < 2, "Could not find base bodies. Please set the base_bodies parameter, or make sure the Model has properly defined legs." );

		m_InitGaitDist = m_PrevGaitDist = GetGaitDist( model );
		m_InitialComPos = model.GetComPos();
	}

	bool GaitMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		// make sure this is a new step
		SCONE_ASSERT( model.GetIntegrationStep() != model.GetPreviousIntegrationStep() );

		// check termination
		auto com_height = model.GetComHeight();
		if ( com_height < termination_height * m_InitialComPos.y )
			return true;

		// update min_velocity measure on new step
		bool new_contact = HasNewFootContact( model );
		TimeInSeconds dt = steps_.empty() ? timestamp : timestamp - steps_.back().time;
		if ( new_contact && dt > min_step_duration )
			AddStep( model, timestamp );

		return false;
	}

	double GaitMeasure::ComputeResult( const Model& model )
	{
		// add final step and penalty to min_velocity measure
		// #todo: only when not at the end of the simulation?
		AddStep( model, model.GetTime() );

		// precompute some values
		double distance = GetGaitDist( model ) - m_InitGaitDist;
		double speed = distance / model.GetTime();
		double duration = model.GetSimulationEndTime();
		size_t step_count = steps_.size();

		// compute measure based on step data
		double step_measure = 0.0;
		double step_length = 0.0;
		double step_duration = 0.0;
		double step_time = 0.0;
		int start_step = xo::clamped( int( step_count ) - initiation_steps, 0, initiation_steps );
		size_t counted_steps = 0;
		for ( int step = 0; step < step_count; ++step )
		{
			double dt = step > 0 ? steps_[step].time - steps_[step - 1].time : steps_[step].time;
			double step_vel = steps_[step].length / dt;
			double step_penalty = Range< double >( min_velocity, max_velocity ).GetRangeViolation( step_vel );
			double norm_vel = xo::clamped( 1.0 - ( fabs( step_penalty ) / min_velocity ), -1.0, 1.0 );

			log::TraceF( "%.3f: step=%d vel=%.3f (%.3f/%.3f) penalty=%.3f norm_vel=%.3f %s",
				steps_[step].time, step, step_vel, steps_[step].length, dt, step_penalty, norm_vel, step < start_step ? "" : "*" );

			if ( step >= start_step )
			{
				step_measure += dt * norm_vel;
				step_time += dt;
				step_length += steps_[step].length;
				step_duration += dt;
				counted_steps++;
			}
		}

		if ( model.GetTime() < duration )
			step_time += duration - model.GetTime();

		// set results
		report_.set( "step_velocity", step_length / step_time );
		report_.set( "step_count", step_count );

		return 1.0 - step_measure / step_time;
	}

	double GaitMeasure::GetCurrentResult( const Model& model )
	{
		double step_vel = steps_.empty() ? 0 : steps_.back().length / GetStepDuration( steps_.size() - 1 );
		double step_penalty = Range< double >( min_velocity, max_velocity ).GetRangeViolation( step_vel );
		double norm_vel = xo::clamped( 1.0 - ( fabs( step_penalty ) / min_velocity ), -1.0, 1.0 );
		return norm_vel;
	}

	void GaitMeasure::Reset( Model& model )
	{
		Measure::Reset( model );
		steps_.clear();
		m_PrevContactState.clear();
		m_PrevGaitDist = 0.0;
		m_Report.clear();
	}

	void GaitMeasure::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		frame["step_length"] = steps_.empty() ? 0 : steps_.back().length;
		frame["step_velocity"] = steps_.empty() ? 0 : steps_.back().length / GetStepDuration( steps_.size() - 1 );
	}

	void GaitMeasure::AddStep( const Model& model, double timestamp )
	{
		double gait_dist = GetGaitDist( model );
		double step_length = gait_dist - m_PrevGaitDist;

		steps_.emplace_back( Step{ model.GetTime(), step_length } );
		m_PrevGaitDist = gait_dist;
	}

	Real GaitMeasure::GetGaitDist( const Model& model )
	{
		// compute average of feet and Com (smallest 2 values)
		SCONE_ASSERT( m_BaseBodies.size() >= 2 );
		xo::sorted_vector< double > distances;
		distances.reserve( 3 );
		distances.insert( xo::dot_product( direction, model.GetComPos() ) );
		distances.insert( xo::dot_product( direction, m_BaseBodies[0]->GetComPos() ) );
		distances.insert( xo::dot_product( direction, m_BaseBodies[1]->GetComPos() ) );
		return ( distances[0] + distances[1] ) / 2;
	}

	Real GaitMeasure::GetStepDuration( index_t step ) const
	{
		if ( steps_.empty() )
			return 0.0;
		else if ( step == 0 )
			return steps_.front().time;
		SCONE_ASSERT( steps_.size() > step );
		return steps_[step].time - steps_[step - 1].time;
	}

	String GaitMeasure::GetClassSignature() const
	{
		return stringf( "S%02d", static_cast<int>( 10 * min_velocity ) );
	}

	bool GaitMeasure::HasNewFootContact( const Model& model )
	{
		if ( m_PrevContactState.empty() )
		{
			// initialize
			for ( const auto& leg : model.GetLegs() )
				m_PrevContactState.push_back( leg.GetLoad() >= load_threshold );
			return false;
		}

		bool has_new_contact = false;
		for ( size_t idx = 0; idx < model.GetLegCount(); ++idx )
		{
			bool contact = model.GetLeg( idx ).GetLoad() >= load_threshold;
			has_new_contact |= contact && !m_PrevContactState[idx];
			m_PrevContactState[idx] = contact;
		}

		return has_new_contact;
	}
}
