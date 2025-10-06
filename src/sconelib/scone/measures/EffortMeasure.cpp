/*
** EffortMeasure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "EffortMeasure.h"
#include "scone/model/Model.h"
#include "scone/model/Dof.h"
#include "scone/model/Muscle.h"
#include "scone/core/profiler_config.h"
#include "scone/core/math.h"
#include "xo/string/pattern_matcher.h"
#include "xo/numerical/math.h"
#include "scone/core/IncludeExcludePattern.h"

namespace scone
{
	xo::dictionary< EffortMeasureType > g_MeasureNames{
		{ EffortMeasureType::Constant, "Constant" },
		{ EffortMeasureType::TotalForce, "TotalForce" },
		{ EffortMeasureType::Wang2012, "Wang2012" },
		{ EffortMeasureType::Uchida2016, "Uchida2016" },
		{ EffortMeasureType::SquaredMuscleStress, "SquaredMuscleStress" },
		{ EffortMeasureType::CubedMuscleStress, "CubedMuscleStress" },
		{ EffortMeasureType::MuscleActivation, "MuscleActivation" },
		{ EffortMeasureType::SquaredMuscleActivation, "SquaredMuscleActivation" },
		{ EffortMeasureType::CubedMuscleActivation, "CubedMuscleActivation" },
		{ EffortMeasureType::MechnicalWork, "MechnicalWork" },
		{ EffortMeasureType::MotorTorque, "MotorTorque" }
	};

	EffortMeasure::EffortMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
		Measure( props, par, model, loc ),
		m_Effort( Statistic<>::LinearInterpolation )
	{
		measure_type = g_MeasureNames( props.get<String>( "measure_type" ) );
		INIT_PROP( props, include, "*" );
		INIT_PROP( props, exclude, "" );
		INIT_PROP( props, use_cost_of_transport, false );
		INIT_PROP( props, specific_tension, 0.25e6 );
		INIT_PROP( props, muscle_density, 1059.7 );
		INIT_PROP( props, default_muscle_slow_twitch_ratio, 0.5 );
		INIT_PROP( props, use_symmetric_fiber_ratios, true );
		INIT_PROP( props, min_distance, 1.0 );
		INIT_PROP( props, use_average_per_muscle, false );
		INIT_PROP( props, use_muscle_volume_weighting, false );
		INIT_PROP( props, omnidirectional, false );
		order = props.get_any( { "mechanical_work_order", "order" }, 1.0 );

		SCONE_ERROR_IF( use_average_per_muscle && use_muscle_volume_weighting, "Cannot use both use_average_per_muscle and use_muscle_volume_weighting" );

		if ( name_.empty() )
			name_ = g_MeasureNames( measure_type );

		// initialize muscle list
		IncludeExcludePattern match{ include, exclude };
		for ( auto* mus : model.GetMuscles() )
			if ( match( mus->GetName() ) )
				m_MusclePtrs.emplace_back( mus );
		SCONE_ERROR_IF( m_MusclePtrs.empty(), "No muscles included in EffortMeasure" );

		// precompute some stuff
		m_Wang2012BasalEnergy = 1.51 * model.GetMass();
		m_Uchida2016BasalEnergy = 1.2 * model.GetMass();
		m_AerobicFactor = 1.5; // 1.5 is for aerobic conditions, 1.0 for anaerobic. may need to add as option later
		m_InitComPos = model.GetComPos() - model.GetGroundBody().GetOriginPos();
		SetSlowTwitchRatios( props, model );
		for ( auto& mus : m_MusclePtrs )
			m_MuscleNames.push_back( name_ + "." + mus->GetName() + ".penalty" );
		m_MuscleEfforts.resize( m_MusclePtrs.size() );
	}

	EffortMeasure::MuscleProperties::MuscleProperties( const PropNode& props ) :
		muscle( props.get< String >( "muscle" ) )
	{
		Real ratio = props.get< Real >( "slow_twitch_ratio" );
		SCONE_ASSERT_MSG( ( ratio >= 0.0 && ratio <= 1.0 ), "slow_twitch_ratios must be between 0.0 and 1.0" );
		slow_twitch_ratio = ratio;
	}

	UpdateResult EffortMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		// make sure this is a new step and the measure is active
		SCONE_ASSERT( model.GetIntegrationStep() != model.GetPreviousIntegrationStep() );

		double current_effort = GetCurrentEffort( model );
		m_Effort.AddSample( timestamp, current_effort );

		return false;
	}

	double EffortMeasure::ComputeResult( const Model& model )
	{
		xo::optional<double> result;
		if ( use_cost_of_transport )
		{
			auto tot_effort = m_Effort.GetTotal();
			auto delta_com = model.GetComPos() - model.GetGroundBody().GetOriginPos() - m_InitComPos;
			double distance = std::max( min_distance, omnidirectional ? xo::length( delta_com ) : std::abs( delta_com.x ) );
			report_.set( "effort", tot_effort );
			report_.set( "distance", distance );
			result = tot_effort / ( model.GetMass() * distance );
		}
		else
		{
			auto avg_effort = m_Effort.GetAverage();
			report_.set( "effort", avg_effort );
			result = avg_effort;
		}

		SCONE_ASSERT( result );
		if ( use_average_per_muscle && !m_MusclePtrs.empty() )
			*result /= m_MusclePtrs.size();

		return *result;
	}

	template<int Order> Real GetMuscleActivation( const std::vector<Muscle*>& muscles, bool use_muscle_volume_weighting ) {
		double sum = 0.0;
		if ( use_muscle_volume_weighting ) {
			double total_vol = 0.0;
			for ( auto& m : muscles ) {
				auto vol = m->GetVolume();
				sum += vol * xo::power<Order>( m->GetActivation() );
				total_vol += vol;
			}
			return sum / total_vol;
		} else {
			for ( auto& m : muscles )
				sum += xo::power<Order>( m->GetActivation() );
			return sum;
		}
	}

	double EffortMeasure::GetCurrentEffort( const Model& model ) const
	{
		switch ( measure_type )
		{
		case EffortMeasureType::TotalForce: return GetTotalForce( model );
		case EffortMeasureType::Wang2012: return GetWang2012( model );
		case EffortMeasureType::Constant: return model.GetMass();
		case EffortMeasureType::Uchida2016: return GetUchida2016( model );
		case EffortMeasureType::SquaredMuscleStress: return GetSquaredMuscleStress( model );
		case EffortMeasureType::CubedMuscleStress: return GetCubedMuscleStress( model );
		case EffortMeasureType::MuscleActivation: return GetMuscleActivation<1>( m_MusclePtrs, use_muscle_volume_weighting );
		case EffortMeasureType::SquaredMuscleActivation: return GetMuscleActivation<2>( m_MusclePtrs, use_muscle_volume_weighting );
		case EffortMeasureType::CubedMuscleActivation: return GetMuscleActivation<3>( m_MusclePtrs, use_muscle_volume_weighting );
		case EffortMeasureType::MechnicalWork: return GetMechnicalWork( model );
		case EffortMeasureType::MotorTorque: return GetMotorTorque( model );
		default: SCONE_THROW( "Invalid energy measure" );
		}
	}

	double EffortMeasure::GetTotalForce( const Model& model ) const
	{
		double f = 1.0; // base muscle force
		for ( const auto& mus : m_MusclePtrs )
			f += mus->GetForce();
		return f;
	}

	double EffortMeasure::GetWang2012( const Model& model ) const
	{
		double e = m_Wang2012BasalEnergy;
		for ( index_t i = 0; i < m_MusclePtrs.size(); ++i )
		{
			const auto& mus = m_MusclePtrs[i];
			double mass = mus->GetMass( specific_tension, muscle_density );
			Real l = m_SlowTwitchFiberRatios[i];
			Real fa = 40 * l * sin( REAL_HALF_PI * mus->GetExcitation() ) + 133 * ( 1 - l ) * ( 1 - cos( REAL_HALF_PI * mus->GetExcitation() ) );
			Real fm = 74 * l * sin( REAL_HALF_PI * mus->GetActivation() ) + 111 * ( 1 - l ) * ( 1 - cos( REAL_HALF_PI * mus->GetActivation() ) );
			Real l_ce_norm = mus->GetFiberLength() / mus->GetOptimalFiberLength();
			Real v_ce = mus->GetFiberVelocity();
			Real g = 0.0;
			if ( l_ce_norm < 0.5 )
				g = 0.5;
			else if ( l_ce_norm < 1.0 )
				g = l_ce_norm;
			else if ( l_ce_norm < 1.5 )
				g = -2 * l_ce_norm + 3;

			Real effort_a = mass * fa;
			Real effort_m = mass * g * fm;
			Real effort_s = xo::max( 0.0, 0.25 * mus->GetForce() * -v_ce );
			Real effort_w = xo::max( 0.0, mus->GetActiveFiberForce() * -v_ce );
			Real effort = effort_a + effort_m + effort_s + effort_w;
			e += effort;
			if ( model.GetStoreData() )
				m_MuscleEfforts[i] = effort;

			SCONE_ERROR_IF( fa != fa, "Error computing fa for " + mus->GetName() + "; excitation=" + to_str( mus->GetExcitation() ) );
		}

		return e;
	}

	// Implementation of Umberger (2003, 2010) metabolics model 
	// with updates from Uchida 2016.
	double EffortMeasure::GetUchida2016( const Model& model ) const
	{
		double e = m_Uchida2016BasalEnergy;
		for ( index_t i = 0; i < m_MusclePtrs.size(); ++i )
		{
			const auto& mus = m_MusclePtrs[i];
			double mass = mus->GetMass( specific_tension, muscle_density );

			// calculate A parameter
			Real excitation = mus->GetExcitation();
			Real activation = mus->GetActivation();
			double A;
			if ( excitation > activation )
				A = excitation;
			else
				A = ( excitation + activation ) / 2;

			// calculate slowTwitchRatio factor
			Real slowTwitchRatio = m_SlowTwitchFiberRatios[i];
			double uSlow = slowTwitchRatio * sin( REAL_HALF_PI * excitation );
			double uFast = ( 1 - slowTwitchRatio ) * ( 1 - cos( REAL_HALF_PI * excitation ) );
			slowTwitchRatio = ( excitation == 0 ) ? 1.0 : uSlow / ( uSlow + uFast );

			// calculate AMdot
			double AMdot;
			double unscaledAMdot = 128 * ( 1 - slowTwitchRatio ) + 25;
			double F_iso = mus->GetActiveForceLengthMultipler();
			if ( mus->GetNormalizedFiberLength() <= 1.0 )
				AMdot = m_AerobicFactor * std::pow( A, 0.6 ) * unscaledAMdot;
			else
				AMdot = m_AerobicFactor * std::pow( A, 0.6 ) * ( ( 0.4 * unscaledAMdot ) + ( 0.6 * unscaledAMdot * F_iso ) );

			// calculate shortening heat rate
			double Sdot;
			double Vmax_fasttwitch = mus->GetMaxContractionVelocity();
			double Vmax_slowtwitch = mus->GetMaxContractionVelocity() / 2.5;
			double alpha_shortening_fasttwitch = 153 / Vmax_fasttwitch;
			double alpha_shortening_slowtwitch = 100 / Vmax_slowtwitch;
			double fiber_velocity_normalized = mus->GetFiberVelocity() / mus->GetOptimalFiberLength();
			double unscaledSdot, tmp_slowTwitch, tmp_fastTwitch;

			if ( fiber_velocity_normalized <= 0 )
			{
				double maxShorteningRate = 100.0; // (W/kg)
				tmp_slowTwitch = -alpha_shortening_slowtwitch * fiber_velocity_normalized;
				if ( tmp_slowTwitch > maxShorteningRate ) tmp_slowTwitch = maxShorteningRate;

				tmp_fastTwitch = alpha_shortening_fasttwitch * fiber_velocity_normalized * ( 1 - slowTwitchRatio );
				unscaledSdot = ( tmp_slowTwitch * slowTwitchRatio ) - tmp_fastTwitch;
				Sdot = m_AerobicFactor * A * A * unscaledSdot;
			}
			else
			{
				unscaledSdot = 4.0 * alpha_shortening_slowtwitch * fiber_velocity_normalized;
				Sdot = m_AerobicFactor * A * unscaledSdot;
			}

			if ( mus->GetNormalizedFiberLength() > 1.0 ) Sdot *= F_iso;

			double active_fiber_force = mus->GetActiveFiberForce();
			if ( active_fiber_force < 0 ) active_fiber_force = 0;

			// calculate mechanical work rate
			double Wdot =
				-active_fiber_force * mus->GetFiberVelocity() / mass;

			// prevent instantaneous negative power by accounting for it through Sdot
			double Edot_Wkg_beforeClamp = AMdot + Sdot + Wdot;
			if ( Edot_Wkg_beforeClamp < 0 ) Sdot -= Edot_Wkg_beforeClamp;

			// total heat rate cannot fall below 1.0 W/kg
			double totalHeatRate = AMdot + Sdot;
			if ( totalHeatRate < 1.0 ) totalHeatRate = 1.0;

			// total metabolic rate for this muscle
			double Edot = ( totalHeatRate + Wdot ) * mass;

			if ( model.GetStoreData() )
				m_MuscleEfforts[i] = Edot;

			e += Edot;
		}
		return e;
	}

	void EffortMeasure::SetSlowTwitchRatios( const PropNode& props, const Model& model )
	{
		// initialize all muscles to default
		std::vector< Real > init( m_MusclePtrs.size(), default_muscle_slow_twitch_ratio );
		m_SlowTwitchFiberRatios = init;

		// read in fiber ratios. throw exception if out of [0,1] range
		//std::map< String, Real > fiberRatioMap;
		std::vector< MuscleProperties > muscPropsInput;
		if ( const PropNode* muscleProperties = props.try_get_child( "MuscleProperties" ) )
		{
			for ( auto it = muscleProperties->begin(); it != muscleProperties->end(); ++it )
				muscPropsInput.emplace_back( MuscleProperties( it->second ) );
		}

		// update muscle if its name is in the map
		for ( index_t i = 0; i < m_MusclePtrs.size(); ++i )
		{
			const auto& mus = m_MusclePtrs[i];

			bool foundMuscle = false;
			for ( auto it = muscPropsInput.begin(); it != muscPropsInput.end(); ++it )
			{
				if ( xo::pattern_matcher( it->muscle, ";" )( mus->GetName() ) )
				{
					SCONE_ASSERT_MSG( !foundMuscle, "multiple muscle names matched in MuscleProperties" );
					m_SlowTwitchFiberRatios[i] = it->slow_twitch_ratio;
					foundMuscle = true;
				}
			}
		}
	}

	double EffortMeasure::GetSquaredMuscleStress( const Model& model ) const
	{
		double sum = 0.0;
		for ( auto& m : m_MusclePtrs )
			sum += xo::squared( m->GetForce() / m->GetPCSA() );
		return sum;
	}

	double EffortMeasure::GetCubedMuscleStress( const Model& model ) const
	{
		double sum = 0.0;
		for ( auto& m : m_MusclePtrs )
			sum += xo::cubed( m->GetForce() / m->GetPCSA() );
		return sum;
	}

	// Implementation of mechanical work, Afschrift et al. 2016, "Mechanical effort predicts the selection of ankle ..." 
	double EffortMeasure::GetMechnicalWork( const Model& model ) const
	{
		double sum = 0.0;
		for ( auto& d : model.GetDofs() )
			sum += std::pow( fabs( d->GetMuscleMoment() * d->GetVel() ), order ); // only positive power
		return sum;
	}

	double EffortMeasure::GetMotorTorque( const Model& model ) const
	{
		double sum = 0.0;
		for ( auto& d : model.GetDofs() )
			if ( d->IsActuated() )
				sum += std::pow( fabs( d->GetActuatorTorque() ), order );
		return sum;
	}

	String EffortMeasure::GetClassSignature() const
	{
		String s;

		switch ( measure_type )
		{
		case EffortMeasureType::TotalForce: s += "F"; break;
		case EffortMeasureType::Wang2012: s += "W"; break;
		case EffortMeasureType::Constant: s += "C"; break;
		case EffortMeasureType::Uchida2016: s += "U"; break;
		case EffortMeasureType::SquaredMuscleStress: s += "S2"; break;
		case EffortMeasureType::CubedMuscleStress: s += "S3"; break;
		case EffortMeasureType::MuscleActivation: s += "A"; break;
		case EffortMeasureType::SquaredMuscleActivation: s += "A2"; break;
		case EffortMeasureType::CubedMuscleActivation: s += "A3"; break;
		case EffortMeasureType::MechnicalWork: s += "MW"; break;
		case EffortMeasureType::MotorTorque: s += "T"; break;
		default: SCONE_THROW( "Invalid energy measure" );
		}

		return s;
	}

	void EffortMeasure::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		frame[name_ + ".penalty"] = m_Effort.GetLatest();
		for ( index_t i = 0; i < m_MuscleEfforts.size(); ++i )
			frame[m_MuscleNames[i]] = m_MuscleEfforts[i];
	}
}
