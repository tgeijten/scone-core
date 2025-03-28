/*
** GaitMeasure.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once
#include "Measure.h"
#include "scone/core/Statistic.h"
#include "EffortMeasure.h"
#include "DofLimitMeasure.h"
#include "xo/utility/optional.h"

namespace scone
{
	/// Measure for locomotion at a predefined speed, defined by the parameters ''min_velocity'' and (optionally) ''max_velocity''.
	/**
	This measure returns a value between ''0'' (velocity conditions are met) and ''1'' (model falls over immediately).

	Specifically, the value represents the average by which the velocity of each step falls between **min_velocity** and **max_velocity**.
	The value is normalized by **min_velocity**, i.e. a step velocity of 0.5 times **min_velocity** has a score of 0.5.

	In order to save simulation time and increasing optimization performance, the ''termination_height'' parameter is used to detect if the model has fallen,
	after which the simulation is terminated early (assuming it will not recover). It does so by comparing the center of mass (COM) height
	to the initial state, and terminates when ''(COM-height / initial-COM-height) < termination_height''.

	See Tutorials 4 and 5 for examples.
	*/
	class GaitMeasure : public Measure
	{
	public:
		GaitMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc );

		/// Relative COM height at which to terminate the simulation, as a factor of the initial COM height; default = 0.5
		Real termination_height;

		/// Measure height with respect to feet instead of ground
		bool use_height_wrt_feet;

		/// Time to continue after fall is detected [s]; default = 0
		TimeInSeconds continue_after_fall;

		/// Minimum velocity [m/s]; default = 0 m/s.
		Real min_velocity;

		/// Optional maximum velocity [m/s]; default = 299792458 m/s.
		Real max_velocity;

		/// Load threshold for step detection; default = 0.1.
		Real load_threshold;

		/// Minimum duration [s] of a step, used for step detection; default = 0.1.
		Real min_step_duration;

		/// Number of initial steps of which the velocity is disregarded in the final measure; default = 2.
		int initiation_steps;

		/// Name of the base bodies (i.e. feet), used for gait detection; default = "toes_l toes_r"
		String base_bodies;

		/// Gait direction vector; default = [ 1 0 0 ]
		Vec3 direction;

		/// Use initial heading for target direction; default = 0
		bool use_initial_heading;

		/// Minimum velocity used for penalty normalization (advanced); default = 0.1 m/s.
		Real min_norm_velocity;

		virtual bool UpdateMeasure( const Model& model, double timestamp ) override;
		void AddStep( const Model& model, double timestamp );
		virtual double ComputeResult( const Model& model ) override;
		virtual double GetCurrentResult( const Model& model ) override;
		virtual void Reset( Model& model ) override;
		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;

	protected:
		virtual String GetClassSignature() const override;

	private:
		std::vector< const Body* > m_BaseBodies;
		Real m_InitComHeight;
		Real m_InitGaitDist;
		xo::optional<TimeInSeconds> m_TerminationTime;

		struct Step {
			TimeInSeconds time;
			Real length;
		};
		std::vector< Step > steps_;
		std::vector< bool > m_PrevContactState;
		Real m_PrevGaitDist;
		PropNode m_Report;

		Real GetNormalizedVelocity( Real p );
		Real GetGaitDist( const Model& model );
		Real GetStepDuration( index_t step ) const;
		bool HasNewFootContact( const Model& model );
	};
}
