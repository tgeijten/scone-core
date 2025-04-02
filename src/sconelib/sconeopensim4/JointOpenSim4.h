/*
** JointOpenSim4.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "platform.h"
#include "scone/model/Joint.h"

namespace OpenSim
{
	class Joint;
}

namespace scone
{
	class SCONE_OPENSIM_4_API JointOpenSim4 : public Joint
	{
	public:
		JointOpenSim4( Body& body, Body& parent, class ModelOpenSim4& model, OpenSim::Joint& osJoint );
		virtual ~JointOpenSim4();

		virtual const String& GetName() const;
		virtual Vec3 GetPos() const override;
		virtual Vec3 GetPosInParent() const override;
		virtual Vec3 GetPosInChild() const override;
		virtual Vec3 GetReactionForce() const override;
		virtual Quat GetOriInParent() const override;
		virtual Quat GetOriInChild() const override;

	private:
		class ModelOpenSim4& m_Model;
		OpenSim::Joint& m_osJoint;
	};
}
