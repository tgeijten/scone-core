/*
** DofOpenSim3.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "DofOpenSim3.h"
#include "ModelOpenSim3.h"
#include "JointOpenSim3.h"
#include "scone/core/Log.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/CoordinateLimitForce.h>
#include "OpenSim/Simulation/SimbodyEngine/CustomJoint.h"
#include "OpenSim/Simulation/SimbodyEngine/PinJoint.h"
#include "OpenSim/Simulation/SimbodyEngine/SpatialTransform.h"
#include <OpenSim/Actuators/CoordinateActuator.h>
#include "scone/core/Angle.h"
#include "simbody_tools.h"

namespace scone
{
	DofOpenSim3::DofOpenSim3( ModelOpenSim3& model, OpenSim::Coordinate& coord ) :
		Dof( FindByName( model.GetJoints(), coord.getJoint().getName() ) ),
		m_Model( model ),
		m_osCoord( coord ),
		m_pOsLimitForce( nullptr ),
		m_OsCoordAct( nullptr ),
		m_RotationAxis()
	{
		// find corresponding CoordinateLimitForce
		auto& forceSet = model.GetOsimModel().getForceSet();
		for ( int idx = 0; idx < forceSet.getSize(); ++idx )
		{
			// OpenSim: Set<T>::get( idx ) is const but returns non-const reference, is this a bug?
			const OpenSim::CoordinateLimitForce* clf = dynamic_cast<const OpenSim::CoordinateLimitForce*>( &forceSet.get( idx ) );
			if ( clf && clf->getProperty_coordinate().getValue() == coord.getName() )
			{
				// we have found a match!
				m_pOsLimitForce = clf;
				break;
			}
		}
	}

	DofOpenSim3::~DofOpenSim3() {}

	Real DofOpenSim3::GetPos() const
	{
		return m_osCoord.getValue( m_Model.GetTkState() );
	}

	Real DofOpenSim3::GetVel() const
	{
		return m_osCoord.getSpeedValue( m_Model.GetTkState() );
	}

	Real DofOpenSim3::GetAcc() const
	{
		return m_osCoord.getAccelerationValue( m_Model.GetTkState() );
	}

	const String& DofOpenSim3::GetName() const
	{
		return m_osCoord.getName();
	}

	Real DofOpenSim3::GetLimitMoment() const
	{
		if ( m_pOsLimitForce )
			return m_pOsLimitForce->calcLimitForce( m_Model.GetTkState() );
		else return 0.0;
	}

	void DofOpenSim3::SetPos( Real pos )
	{
		if ( !m_osCoord.getLocked( m_Model.GetTkState() ) )
			m_osCoord.setValue( m_Model.GetTkState(), pos, false );
	}

	void DofOpenSim3::SetVel( Real vel )
	{
		if ( !m_osCoord.getLocked( m_Model.GetTkState() ) )
			m_osCoord.setSpeedValue( m_Model.GetTkState(), vel );
	}

	bool DofOpenSim3::IsRotational() const
	{
		return m_osCoord.getMotionType() == OpenSim::Coordinate::Rotational;
	}

	Vec3 DofOpenSim3::GetRotationAxis() const
	{
		return m_RotationAxis;
	}

	Vec3 DofOpenSim3::GetLocalAxis() const
	{
		auto* osJoint = &m_osCoord.getJoint();
		if ( auto* customJoint = dynamic_cast<const OpenSim::CustomJoint*>( osJoint ) ) {
			auto& st = customJoint->getSpatialTransform();
			auto myIdx = st.getCoordinateNames().findIndex( m_osCoord.getName() );
			auto idxVec = st.getCoordinateIndices();
			for ( index_t axisIdx = 0; axisIdx < idxVec.size(); ++axisIdx )
				if ( xo::contains( idxVec[axisIdx], myIdx ) )
					return from_osim( st.getAxes()[axisIdx] );
		}
		else if ( auto* pinJoint = dynamic_cast<const OpenSim::PinJoint*>( osJoint ) ) {
			SimTK::Vec3 ori;
			pinJoint->getOrientation( ori );
			SimTK::Rotation rot;
			rot.setRotationToBodyFixedXYZ( ori );
			auto axis = rot * SimTK::Vec3( 0, 0, 1 );
			return from_osim( axis );
		}

		return Vec3::zero();
	}

	Range< Real > DofOpenSim3::GetRange() const
	{
		return Range< Real >( m_osCoord.get_range( 0 ), m_osCoord.get_range( 1 ) );
	}

	Real DofOpenSim3::GetDefaultPos() const
	{
		return m_osCoord.getDefaultValue();
	}

	Real DofOpenSim3::GetMinInput() const
	{
		return m_OsCoordAct ? m_OsCoordAct->getMinControl() : 0.0;
	}

	Real DofOpenSim3::GetMaxInput() const
	{
		return m_OsCoordAct ? m_OsCoordAct->getMaxControl() : 0.0;
	}

	Real DofOpenSim3::GetMinTorque() const
	{
		return m_OsCoordAct ? m_OsCoordAct->getMinControl() * m_OsCoordAct->getOptimalForce() : 0.0;
	}

	Real DofOpenSim3::GetMaxTorque() const
	{
		return m_OsCoordAct ? m_OsCoordAct->getMaxControl() * m_OsCoordAct->getOptimalForce() : 0.0;
	}

	Real DofOpenSim3::GetActuatorTorque() const
	{
		return GetInput() * m_OsCoordAct->getOptimalForce();
	}

	const Model& DofOpenSim3::GetModel() const
	{
		return m_Model;
	}

	PropNode DofOpenSim3::GetInfo() const
	{
		PropNode pn = Dof::GetInfo();
		if ( m_pOsLimitForce ) {
			pn["limit_stiffness"] = m_pOsLimitForce->get_lower_stiffness();
			pn["limit_damping"] = m_pOsLimitForce->get_damping();
			pn["limit_range"] = BoundsDeg( m_pOsLimitForce->get_lower_limit(), m_pOsLimitForce->get_upper_limit() );
		}
		return pn;
	}
}

