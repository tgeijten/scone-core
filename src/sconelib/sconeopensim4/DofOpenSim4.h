/*
** DofOpenSim4.h
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "platform.h"
#include "scone/model/Dof.h"

namespace OpenSim
{
	class Coordinate;
	class CoordinateLimitForce;
	class CoordinateActuator;
}

namespace scone
{
	class SCONE_OPENSIM_4_API DofOpenSim4 : public Dof
	{
	public:
		DofOpenSim4( class ModelOpenSim4& model, OpenSim::Coordinate& coord );
		virtual ~DofOpenSim4();

		virtual Real GetPos() const override;
		virtual Real GetVel() const override;
		virtual Real GetAcc() const override;

		virtual Real GetLimitMoment() const override;

		virtual const String& GetName() const override;
		const OpenSim::Coordinate& GetOsCoordinate() const { return m_osCoord; }

		void SetCoordinateActuator( OpenSim::CoordinateActuator* ca ) { m_OsCoordAct = ca; }

		virtual void SetPos( Real pos ) override;
		virtual void SetVel( Real vel ) override;

		virtual bool IsRotational() const override;
		virtual Vec3 GetRotationAxis() const override;
		virtual Vec3 GetLocalAxis() const override;
		virtual Range< Real > GetRange() const override;
		virtual Real GetDefaultPos() const override;

		virtual bool IsActuated() const override { return m_OsCoordAct != nullptr; }
		virtual Real GetMinInput() const override;
		virtual Real GetMaxInput() const override;
		virtual Real GetMinTorque() const override;
		virtual Real GetMaxTorque() const override;
		virtual Real GetActuatorTorque() const override;

		const Model& GetModel() const override;

		virtual PropNode GetInfo() const override;

	private:
		friend class ModelOpenSim4;
		ModelOpenSim4& m_Model;
		OpenSim::Coordinate& m_osCoord;
		const OpenSim::CoordinateLimitForce* m_pOsLimitForce;
		const OpenSim::CoordinateActuator* m_OsCoordAct;
		Vec3 m_RotationAxis;
	};
}
