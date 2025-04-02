/*
** JointOpenSim3.h
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
	class SCONE_OPENSIM_3_API JointOpenSim3 : public Joint
	{
	public:
		JointOpenSim3( Body& body, Body& parent, class ModelOpenSim3& model, OpenSim::Joint& osJoint );
		virtual ~JointOpenSim3();

		virtual const String& GetName() const;
		virtual Vec3 GetPos() const override;
		virtual Vec3 GetPosInParent() const override;
		virtual Vec3 GetPosInChild() const override;
		virtual Quat GetOriInParent() const override;
		virtual Quat GetOriInChild() const override;
		virtual Vec3 GetReactionForce() const override;

	private:
		class ModelOpenSim3& m_Model;
		OpenSim::Joint& m_osJoint;
	};
}
