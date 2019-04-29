/*
** JointLoadMeasure.h
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Measure.h"
#include "scone/core/Statistic.h"
#include "scone/model/Joint.h"
#include "RangePenalty.h"

namespace scone
{
	/// Measure for penalizing joint load, if above a specific treashold.
	class JointLoadMeasure : public Measure, public RangePenalty< Real >
	{
	public:
		JointLoadMeasure( const PropNode& props, Params& par, Model& model, const Location& loc );
		virtual ~JointLoadMeasure() {}

		virtual double ComputeResult( Model& model ) override;
		virtual bool UpdateMeasure( const Model& model, double timestamp ) override;

	protected:
		virtual String GetClassSignature() const override;
		virtual void StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const override;

	private:
		enum Method { NoMethod, JointReactionForce };
		int method;
		Real joint_load;
		Joint& joint;
	};
}
