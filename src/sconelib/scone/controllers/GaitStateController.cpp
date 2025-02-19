/*
** GaitStateController.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "GaitStateController.h"
#include "scone/model/Model.h"
#include "scone/model/Leg.h"
#include "scone/model/Muscle.h"

#include "scone/model/Body.h"
#include "scone/core/Log.h"
#include "scone/core/profiler_config.h"
#include "scone/model/SensorDelayAdapter.h"
#include "scone/core/Factories.h"
#include "scone/model/Sensors.h"

namespace scone
{
	StringMap< GaitStateController::GaitState > GaitStateController::m_StateNames = StringMap< GaitStateController::GaitState >(
		GaitStateController::UnknownState, "Unknown",
		GaitStateController::EarlyStanceState, "EarlyStance",
		GaitStateController::LateStanceState, "LateStance",
		GaitStateController::LiftoffState, "Liftoff",
		GaitStateController::SwingState, "Swing",
		GaitStateController::LandingState, "Landing"
		);

	GaitStateController::LegState::LegState( Model& m, Leg& l, const PropNode& props, Params& par ) :
		leg( l ),
		load_sensor( m.AcquireDelayedSensor< LegLoadSensor >( l ) ),
		state( UnknownState ),
		leg_load(),
		sagittal_pos( 0.0 ),
		coronal_pos( 0.0 ),
		allow_stance_transition( false ),
		allow_swing_transition( false ),
		allow_late_stance_transition( false ),
		allow_liftoff_transition( false ),
		allow_landing_transition( false ),
		leg_length( props.get( "override_leg_length", l.GetLength() ) ),
		INIT_PAR_MEMBER( props, par, stance_load_threshold, 0.1 ),
		INIT_PAR_MEMBER( props, par, swing_load_threshold, stance_load_threshold ),
		INIT_PAR_MEMBER( props, par, landing_threshold, 0.0 ),
		INIT_PAR_MEMBER( props, par, late_stance_threshold, 0.0 ),
		INIT_PAR_MEMBER( props, par, liftoff_threshold, -1.0 ) // default value is such that parameter has no effect
	{}

	GaitStateController::GaitStateController( const PropNode& props, Params& par, Model& model, const Location& target_area ) :
		Controller( props, par, model, target_area ),
		landing_threshold( 0 ), // not used internally, only for documentation purpose
		late_stance_threshold( 0 ), // not used internally, only for documentation purpose
		liftoff_threshold( 0 ), // not used internally, only for documentation purpose
		override_leg_length( 0 ), // not used internally, only for documentation purpose
		INIT_MEMBER( props, leg_load_sensor_delay, 0.0 ),
		stance_load_threshold( 0 ), // not used internally, only for documentation purpose
		swing_load_threshold( 0 ), // not used internally, only for documentation purpose
		INIT_MEMBER( props, use_model_com_reference_pos, false ),
		INIT_MEMBER( props, symmetric, true ),
		INIT_MEMBER( props, omnidirectional, false )
	{
		// show a helpful error message when no legs are defined
		if ( model.GetLegs().empty() )
		{
			string msg = "GaitStateController could not find any legs in " + model.GetName() + ".\n\n";
			msg += "Please make sure the following values are setup correctly in the Model:\n";
			msg += " - leg_upper_body (default is 'femur')\n";
			msg += " - leg_lower_body (default is two bodies below leg_upper_body)\n";
			msg += " - leg_contact_force (default is 'foot')\n";
			SCONE_ERROR( msg );
		}

		if ( !xo::is_even( model.GetLegCount() ) )
			SCONE_ERROR( "GaitStateController requires model to have an even number of legs. Number of legs found: " + xo::to_str( model.GetLegCount() ) );

		// create leg states
		for ( auto& leg : model.GetLegs() )
		{
			ScopedParamSetPrefixer prefixer( par, symmetric ? "" : leg.GetName() + '.' );
			m_LegStates.push_back( LegState( model, leg, props, par ) );
			//log::TraceF( "leg %d leg_length=%.5f", m_LegStates.back()->leg.GetIndex(), m_LegStates.back()->leg_length );
		}

		// create instances for each controller
		const PropNode& ccProps = props.get_child( "ConditionalControllers" );
		for ( PropNode::const_iterator ccIt = ccProps.begin(); ccIt != ccProps.end(); ++ccIt )
		{
			// get state masks
			String state_masks = ccIt->second.get< String >( "states" );
			auto state_tokens = xo::split_str( state_masks, ";," );
			auto target_side = ccIt->second.get<Side>( "legs", Side::None );
			for ( const String& instance_states_str : state_tokens )
			{
				// automatically create controllers for all legs (sides)
				for ( size_t legIdx = 0; legIdx < model.GetLegs().size(); ++legIdx )
				{
					const auto& leg = model.GetLeg( legIdx );
					if ( target_side != Side::None && leg.GetSide() != target_side )
						continue; // skip this leg

					// create new conditional controller
					m_ConditionalControllers.push_back( ConditionalController() );
					ConditionalController& cc = m_ConditionalControllers.back();

					// initialize state_mask based on names in instance_states_str
					auto instance_states = xo::split_str( instance_states_str, " " );
					for ( const auto& state : instance_states )
						cc.state_mask.set( m_StateNames.GetValue( state ) );
					SCONE_THROW_IF( !cc.state_mask.any(), "Conditional Controller has empty state mask" );

					// initialize leg index
					cc.leg_index = legIdx;

					// #todo: allow neater definition of target loc instead of just taking the leg side
					Location l = Location( model.GetLeg( cc.leg_index ).GetSide(), symmetric );

					// create controller
					//log::trace( "Creating controllers for " + GetConditionName( cc ) );
					auto fp = FindFactoryProps( GetControllerFactory(), ccIt->second, "Controller" );
					//const PropNode& cprops = ccIt->second.get_child( "Controller" );
					ScopedParamSetPrefixer prefixer( par, "S" + cc.state_mask.to_string() + "." );
					cc.controller = CreateController( fp, par, model, l );
				}
			}
		}
	}

	GaitStateController::~GaitStateController()
	{
	}

	bool GaitStateController::ComputeControls( Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		if ( model.GetIntegrationStep() != model.GetPreviousIntegrationStep() )
		{
			// only update the states after a successful integration step
			UpdateLegStates( model, timestamp );
			UpdateControllerStates( model, timestamp );
		}

		for ( ConditionalController& cc : m_ConditionalControllers )
		{
			if ( cc.active )
			{
				//log::Trace( "Updating Controls of " + GetConditionName( *cc ) );
				cc.controller->UpdateControls( model, timestamp - cc.active_since );
			}
		}

		return false;
	}

	void GaitStateController::UpdateLegStates( Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		// update statuses
		for ( size_t idx = 0; idx < m_LegStates.size(); ++idx )
		{
			LegState& ls = m_LegStates[idx];
			ls.leg_load = ls.load_sensor.GetValue( leg_load_sensor_delay );
			ls.allow_stance_transition = ls.load_sensor.GetValue( leg_load_sensor_delay ) > ls.stance_load_threshold;
			ls.allow_swing_transition = ls.load_sensor.GetValue( leg_load_sensor_delay ) <= ls.swing_load_threshold;
			auto reference_pos = use_model_com_reference_pos ? model.GetComPos() : ls.leg.GetBaseBody().GetComPos();
			if ( omnidirectional && model.HasRootBody() ) {
				auto root_ori = model.GetRootBody().GetOrientation();
				Vec3 sag_dir = root_ori * Vec3::unit_x(), cor_dir = root_ori * Vec3::unit_z();
				ls.sagittal_pos = xo::dot_product( sag_dir, ls.leg.GetFootBody().GetComPos() - reference_pos );
				ls.coronal_pos = xo::dot_product( cor_dir, ls.leg.GetFootBody().GetComPos() - reference_pos );
			}
			else {
				ls.sagittal_pos = ls.leg.GetFootBody().GetComPos().x - reference_pos.x;
				ls.coronal_pos = ls.leg.GetFootBody().GetComPos().z - reference_pos.z;
			}
			ls.allow_late_stance_transition = ls.sagittal_pos < ls.leg_length* ls.late_stance_threshold;
			ls.allow_liftoff_transition = ls.sagittal_pos < ls.leg_length* ls.liftoff_threshold;
			ls.allow_landing_transition = ls.sagittal_pos > ls.leg_length * ls.landing_threshold;
		}

		// update states
		for ( size_t idx = 0; idx < m_LegStates.size(); ++idx )
		{
			LegState& ls = m_LegStates[idx];
			LegState& mir_ls = m_LegStates[idx ^ 1];
			GaitState new_state = ls.state;

			switch ( ls.state )
			{
			case UnknownState:
				// initialize state
				if ( ls.allow_stance_transition )
				{
					if ( mir_ls.allow_stance_transition && ls.sagittal_pos < mir_ls.sagittal_pos )
						new_state = LiftoffState;
					else if ( ls.allow_late_stance_transition )
						new_state = LateStanceState;
					else new_state = EarlyStanceState;
				}
				else
				{
					if ( ls.allow_landing_transition )
						new_state = LandingState;
					else new_state = SwingState;
				}
				break;

			case EarlyStanceState:
				// --> liftoff if other leg is eligible for stance and before this leg
				// --> late stance if a position threshold has passed
				if ( mir_ls.allow_stance_transition && ls.sagittal_pos < mir_ls.sagittal_pos )
					new_state = LiftoffState;
				else if ( ls.allow_late_stance_transition )
					new_state = LateStanceState;
				break;

			case LateStanceState:
				// --> liftoff if other leg is eligible for stance and before this leg
				// --> liftoff if position is beyond swing_threshold
				if ( mir_ls.allow_stance_transition && ls.sagittal_pos < mir_ls.sagittal_pos )
					new_state = LiftoffState;
				else if ( ls.allow_liftoff_transition )
					new_state = LiftoffState;
				break;

			case LiftoffState:
				// --> swing if leg load is below threshold
				if ( ls.allow_swing_transition )
					new_state = SwingState;
				break;

			case SwingState:
				// --> early stance if leg load is above threshold
				// --> landing if position is beyond landing_threshold
				if ( ls.allow_stance_transition && ls.sagittal_pos > mir_ls.sagittal_pos )
					new_state = EarlyStanceState;
				if ( !ls.allow_stance_transition && ls.allow_landing_transition )
					new_state = LandingState;
				break;

			case LandingState:
				// --> early stance if leg load is beyond threshold
				if ( ls.allow_stance_transition )
					new_state = EarlyStanceState;
				break;

			case StateCount:
				break;
			}

			if ( new_state != ls.state )
			{
				//log::TraceF( "%.3f: Leg %d state changed from %s to %s", timestamp, idx, ls.GetStateName().c_str(), m_StateNames.GetString( new_state ).c_str() );
				ls.state = new_state;
			}
		}
	}

	void GaitStateController::UpdateControllerStates( Model& model, double timestamp )
	{
		// update controller states
		for ( ConditionalController& cc : m_ConditionalControllers )
		{
			bool activate = cc.state_mask.test( m_LegStates[cc.leg_index].state );

			// activate or deactivate controller
			if ( activate != cc.active )
			{
				cc.active = activate;
				cc.active_since = timestamp;
			}
		}
	}

	String GaitStateController::GetClassSignature() const
	{
#ifdef SCONE_VERBOSE_SIGNATURES
		String s = "G";
		std::map< String, int > controllers;
		for ( const ConditionalController& cc : m_ConditionalControllers )
			controllers[cc.controller->GetSignature()] += 1;
		for ( auto it = controllers.begin(); it != controllers.end(); ++it )
			s += to_str( it->second / m_LegStates.size() ) + it->first;
		return s;
#else
		return "";
#endif
	}

	void GaitStateController::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		// store states
		for ( size_t idx = 0; idx < m_LegStates.size(); ++idx )
			frame[m_LegStates[idx].leg.GetName() + ".state"] = m_LegStates[idx].state;

		// store sagittal pos
		for ( size_t idx = 0; idx < m_LegStates.size(); ++idx )
			frame[m_LegStates[idx].leg.GetName() + ".sag_pos"] = m_LegStates[idx].sagittal_pos;

		for ( auto& cc : m_ConditionalControllers )
		{
			if ( cc.active )
				cc.controller->StoreData( frame, flags );
		}
	}

	int GaitStateController::TrySetControlParameter( const String& name, Real value )
	{
		int result = 0;
		for ( auto& cc : m_ConditionalControllers )
			result += cc.controller->TrySetControlParameter( name, value );
		return result;
	}

	xo::optional<Real> GaitStateController::TryGetControlParameter( const String& name )
	{
		for ( auto& cc : m_ConditionalControllers )
			if ( auto v = cc.controller->TryGetControlParameter( name ) )
				return *v;
		return {};
	}

	std::vector<String> GaitStateController::GetControlParameters() const
	{
		std::vector<String> results;
		for ( auto& cc : m_ConditionalControllers )
			xo::append( results, cc.controller->GetControlParameters() );
		return results;
	}

	void GaitStateController::Reset( Model& model )
	{
		for ( auto& ls : m_LegStates )
			ls.Reset();
		for ( auto& cc : m_ConditionalControllers )
			cc.Reset();
	}

	String GaitStateController::GetConditionName( const ConditionalController& cc ) const
	{
		String s = m_LegStates[cc.leg_index].leg.GetName();
		for ( int i = 0; i < StateCount; ++i )
		{
			if ( cc.state_mask.test( i ) )
				s += "-" + m_StateNames.GetString( GaitState( i ) );
		}
		return s;
	}
}
