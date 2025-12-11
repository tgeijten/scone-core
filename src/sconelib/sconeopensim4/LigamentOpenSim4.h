/*
** LigamentOpenSim4.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "platform.h"
#include "scone/model/Ligament.h"

namespace OpenSim
{
	class Ligament;
}

namespace scone
{
	class ModelOpenSim4;

	class SCONE_OPENSIM_4_API LigamentOpenSim4 : public Ligament
	{
	public:
		LigamentOpenSim4( ModelOpenSim4& model, const OpenSim::Ligament& mus );
		virtual ~LigamentOpenSim4();

		const Body& GetOriginBody() const override;
		const Body& GetInsertionBody() const override;
		const Model& GetModel() const override;

		Real GetMomentArm( const Dof& dof ) const override;
		Real GetPcsaForce() const override;
		Real GetRestingLength() const override;

		Real GetForce() const override;
		Real GetNormalizedForce() const override;

		Real GetLength() const override;
		Real GetNormalizedLength() const override;

		Real GetVelocity() const override;
		Real GetNormalizedVelocity() const override;

		std::vector<Vec3> GetLigamentPath() const override;
		std::vector<PathElement> GetLocalLigamentPath() const override;

		const String& GetName() const override;

	private:
		ModelOpenSim4& m_Model;
		const OpenSim::Ligament& m_osMus;
	};
}
