/*
** MuscleOpenSim4.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>

#include "MuscleOpenSim4.h"
#include "ModelOpenSim4.h"

#include "scone/core/Exception.h"
#include "scone/core/profiler_config.h"

#include "DofOpenSim4.h"
#include "simbody_tools.h"
#include "xo/numerical/math.h"

namespace scone
{
	const double MOMENT_ARM_EPSILON = 0.000001;

	MuscleOpenSim4::MuscleOpenSim4( ModelOpenSim4& model, OpenSim::Muscle& mus ) :
		Muscle( model ),
		m_Model( model ),
		m_osMus( mus )
	{
		InitJointsDofs();

		// initialize m_MinActivation (#opensim Muscle does not have an interface for this)
		if ( auto mus = dynamic_cast<OpenSim::Millard2012EquilibriumMuscle*>( &m_osMus ) )
			m_MinActivation = mus->getMinimumActivation();
		else if ( auto mus = dynamic_cast<OpenSim::Thelen2003Muscle*>( &m_osMus ) )
			m_MinActivation = mus->getMinimumActivation();
		else m_MinActivation = m_osMus.getMinControl();

		InitializeActivation( model.initial_equilibration_activation );
	}

	MuscleOpenSim4::~MuscleOpenSim4()
	{}

	const String& MuscleOpenSim4::GetName() const
	{
		return m_osMus.getName();
	}

	Real MuscleOpenSim4::GetOptimalFiberLength() const
	{
		return m_osMus.getOptimalFiberLength();
	}

	Real MuscleOpenSim4::GetTendonSlackLength() const
	{
		return m_osMus.getTendonSlackLength();
	}

	Real MuscleOpenSim4::GetPennationAngleAtOptimal() const
	{
		return m_osMus.getPennationAngleAtOptimalFiberLength();
	}

	Real MuscleOpenSim4::GetForce() const
	{
		// OpenSim: why can't I just use getWorkingState()?
		// OpenSim: why must I update to Dynamics for getForce()?
		m_Model.GetOsimModel().getMultibodySystem().realize( m_Model.GetTkState(), SimTK::Stage::Velocity );
		// #osim4: is this seriously how I get the muscle force?
		return m_osMus.getActuation( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetNormalizedForce() const
	{
		return GetForce() / GetMaxIsometricForce();
	}

	Real MuscleOpenSim4::GetLength() const
	{
		m_Model.GetOsimModel().getMultibodySystem().realize( m_Model.GetTkState(), SimTK::Stage::Position );
		return m_osMus.getLength( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetVelocity() const
	{
		m_Model.GetOsimModel().getMultibodySystem().realize( m_Model.GetTkState(), SimTK::Stage::Velocity );
		return m_osMus.getLengtheningSpeed( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetFiberForce() const
	{
		return m_osMus.getFiberForce( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetActiveFiberForce() const
	{
		return m_osMus.getActiveFiberForce( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetPassiveFiberForce() const
	{
		return m_osMus.getPassiveFiberForce( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetFiberLength() const
	{
		return m_osMus.getFiberLength( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetNormalizedFiberLength() const
	{
		m_Model.GetOsimModel().getMultibodySystem().realize( m_Model.GetTkState(), SimTK::Stage::Position );
		return m_osMus.getNormalizedFiberLength( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetCosPennationAngle() const
	{
		return m_osMus.getCosPennationAngle( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetFiberVelocity() const
	{
		return m_osMus.getFiberVelocity( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetNormalizedFiberVelocity() const
	{
		return m_osMus.getFiberVelocity( m_Model.GetTkState() ) / m_osMus.getOptimalFiberLength();
	}

	const Body& MuscleOpenSim4::GetOriginBody() const
	{
		auto& pps = m_osMus.getGeometryPath().getPathPointSet();
		return *FindByName( m_Model.GetBodies(), pps.get( 0 ).getBodyName() );
	}

	const Body& MuscleOpenSim4::GetInsertionBody() const
	{
		auto& pps = m_osMus.getGeometryPath().getPathPointSet();
		return *FindByName( m_Model.GetBodies(), pps.get( pps.getSize() - 1 ).getBodyName() );
	}

	Real MuscleOpenSim4::GetMomentArm( const Dof& dof ) const
	{
#if ENABLE_MOMENT_ARM_CACHE
		auto t = GetModel().GetTime();
		if ( m_MomentArmCacheTimeStamp != t )
		{
			for ( auto& d : GetDofs() )
			{
				const DofOpenSim4& dof_sb = dynamic_cast<const DofOpenSim4&>( *d );
				auto mom = m_osMus.getGeometryPath().computeMomentArm( m_Model.GetTkState(), dof_sb.GetOsCoordinate() );
				if ( fabs( mom ) < MOMENT_ARM_EPSILON || dof_sb.GetOsCoordinate().getLocked( m_Model.GetTkState() ) )
					mom = 0;
				m_MomentArmCache[&dof] = mom;
			}
		}
		return m_MomentArmCache[&dof];
#else
		const auto& dof_sb = dynamic_cast<const DofOpenSim4&>( dof );
		return m_osMus.getGeometryPath().computeMomentArm( m_Model.GetTkState(), dof_sb.GetOsCoordinate() );
#endif
	}

	void MuscleOpenSim4::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		Muscle::StoreData( frame, flags );
		if ( flags.get<StoreDataTypes::DebugData>() )
		{
			auto f_t = m_osMus.getTendonForce( m_Model.GetTkState() ) / m_osMus.getCosPennationAngle( m_Model.GetTkState() ) / m_osMus.getMaxIsometricForce();
			auto f_pe = m_osMus.getPassiveFiberForce( m_Model.GetTkState() ) / m_osMus.getMaxIsometricForce();
			auto f_ce = m_osMus.getActiveForceLengthMultiplier( m_Model.GetTkState() ) * m_osMus.getActivation( m_Model.GetTkState() );
			frame[GetName() + ".inv_ce_vel"] = ( f_t - f_pe ) / f_ce;
			frame[GetName() + ".ce_vel_norm"] = m_osMus.getNormalizedFiberVelocity( m_Model.GetTkState() );
			frame[GetName() + ".ce_vel"] = m_osMus.getFiberVelocity( m_Model.GetTkState() );
			frame[GetName() + ".inv_ce_vel_ft"] = f_t;
			frame[GetName() + ".inv_ce_vel_fpe"] = f_pe;
			frame[GetName() + ".inv_ce_vel_fce"] = f_ce;
		}
	}

	const Model& MuscleOpenSim4::GetModel() const
	{
		return m_Model;
	}

	Real MuscleOpenSim4::GetTendonLength() const
	{
		return m_osMus.getTendonLength( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetNormalizedTendonLength() const
	{
		return m_osMus.getTendonLength( m_Model.GetTkState() ) / m_osMus.getTendonSlackLength();
	}

	Real MuscleOpenSim4::GetActiveForceLengthMultipler() const
	{
		return m_osMus.getActiveForceLengthMultiplier( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetForceVelocityMultipler() const
	{
		return m_osMus.getForceVelocityMultiplier( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetMaxContractionVelocity() const
	{
		return m_osMus.getMaxContractionVelocity();
	}

	Real MuscleOpenSim4::GetTendonStrainAtOneNormForce() const
	{
		if ( auto* m = dynamic_cast<OpenSim::Millard2012EquilibriumMuscle*>( &m_osMus ) )
			return m->getTendonForceLengthCurve().getStrainAtOneNormForce();
		else if ( auto* m = dynamic_cast<OpenSim::DeGrooteFregly2016Muscle*>( &m_osMus ) )
			return m->get_tendon_strain_at_one_norm_force();
		else return 0.0;
	}

	Real MuscleOpenSim4::GetPassiveFiberStrainAtOneNormForce() const
	{
		if ( auto* m = dynamic_cast<OpenSim::Millard2012EquilibriumMuscle*>( &m_osMus ) )
			return m->getFiberForceLengthCurve().getStrainAtOneNormForce();
		else if ( auto* m = dynamic_cast<OpenSim::DeGrooteFregly2016Muscle*>( &m_osMus ) )
			return m->get_passive_fiber_strain_at_one_norm_force();
		else return 0.0;
	}

	Real MuscleOpenSim4::GetMaxIsometricForce() const
	{
		return m_osMus.getMaxIsometricForce();
	}

	std::vector< Vec3 > MuscleOpenSim4::GetMusclePath() const
	{
		//m_Model.GetOsimModel().getMultibodySystem().realize( m_Model.GetTkState(), SimTK::Stage::Velocity );
		//m_osMus.getGeometryPath().updateGeometry( m_Model.GetTkState() );
		auto& pps = m_osMus.getGeometryPath().getCurrentPath( m_Model.GetTkState() );
		std::vector< Vec3 > points( pps.getSize() );
		for ( int i = 0; i < points.size(); ++i )
		{
			const auto& mob = m_Model.GetOsimModel().getMultibodySystem().getMatterSubsystem().getMobilizedBody( pps[i]->getBody().getMobilizedBodyIndex() );
			auto world_pos = mob.getBodyTransform( m_Model.GetTkState() ) * pps[i]->getLocation( m_Model.GetTkState() );

			points[i] = from_osim( world_pos );
		}
		return points;
	}

	std::vector<PathElement> MuscleOpenSim4::GetLocalMusclePath() const
	{
		std::vector<PathElement> points;

#if 0
		// we use getCurrentPath() because that makes it easier to see the index of the wrap object
		// #todo: use getPointSet() and getWrapSet() to get the points and figure out where to insert wrap objects
		auto& pps = m_osMus.getGeometryPath().getCurrentPath( m_Model.GetTkState() );
		int wc_count = 0;
		for ( int i = 0; i < pps.size(); ++i ) {
			auto* b = FindByName( m_Model.GetBodies(), pps[i]->getBody().getName() );
			if ( auto* wc = dynamic_cast<const OpenSim::WrapCylinder*>( pps[i]->getWrapObject() ) ) {
				auto pos = from_osim( wc->get_translation() );
				auto dir = from_osim_euler_xyz( wc->get_xyz_body_rotation() ) * Vec3::unit_z();
				points.emplace_back( b, pos, dir, wc->get_radius() );
				++i, ++wc_count; // skip next point because wrap objects create 2 points, also keep count
			}
			else {
				auto pos = from_osim( pps[i]->getLocation( m_Model.GetTkState() ) );
				points.emplace_back( b, pos );
			}
		}

		// check if there are remaining wrapping surfaces
		auto& ws = m_osMus.getGeometryPath().getWrapSet();
		if ( ws.getSize() > wc_count ) {
			log::warning( "Not all wrapping objects where converted" );
			for ( int i = 0; i < ws.getSize(); ++i ) {
				auto* w = ws[i].getWrapObject();
				auto& f = w->getFrame();
				log::info( GetName(), " body=", f.getName(), " wrap=", w->getName(), " ", ws[i].getStartPoint(), " ", ws[i].getEndPoint() );
			}
		}
#else
		// add path points
		auto& ps = m_osMus.getGeometryPath().getPathPointSet();
		for ( int i = 0; i < ps.getSize(); ++i ) {
			auto* b = FindByName( m_Model.GetBodies(), ps[i].getBody().getName() );
			auto pos = from_osim( ps[i].getLocation( m_Model.GetTkState() ) );
			points.emplace_back( b, pos );
			//log::info( GetName(), "\t", i, "\tbody=", b->GetName() );
		}

		// insert wrap objects
		auto& ws = m_osMus.getGeometryPath().getWrapSet();
		for ( int i = 0; i < ws.getSize(); ++i ) {
			if ( auto* wc = dynamic_cast<const OpenSim::WrapCylinder*>( ws[i].getWrapObject() ) ) {
				auto* b = FindByName( m_Model.GetBodies(), wc->getFrame().getName() );
				auto pos = from_osim( wc->get_translation() );
				bool flip = xo::str_equals_any_of( wc->get_quadrant(), { "-x", "-y" } );
				auto dir = from_osim_euler_xyz( wc->get_xyz_body_rotation() ) * ( flip ? Vec3::neg_unit_z() : Vec3::unit_z() );
				//log::info( GetName(), "\t", i, "\tbody=", b->GetName(), " ", ws[i].getStartPoint(), " ", ws[i].getEndPoint() );
				for ( int j = 1; j < points.size(); ++j ) {
					if ( points[j - 1].body->GetName() == b->GetName() && points[j].body->GetName() != b->GetName() ) {
						points.insert( points.begin() + j, PathElement{ b, pos, dir, wc->get_radius() } );
						++j;
					}
				}
			}
		}

#endif
		return points;
	}

	Real MuscleOpenSim4::GetActivation() const
	{
		return m_osMus.getActivation( m_Model.GetTkState() );
	}

	Real MuscleOpenSim4::GetExcitation() const
	{
		// use our own control value, as OpenSim calls getControls()
		// this could lead to infinite recursion
		// make sure to clamp it for calls (important for metabolics)
		return xo::clamped( GetInput(), m_MinActivation, m_MaxActivation );
	}

	void MuscleOpenSim4::SetExcitation( Real u )
	{
		m_osMus.setExcitation( m_Model.GetTkState(), u );
	}

	void MuscleOpenSim4::InitializeActivation( Real a )
	{
		m_osMus.setExcitation( m_Model.GetTkState(), a );
		m_osMus.setActivation( m_Model.GetTkState(), a );
	}
}
