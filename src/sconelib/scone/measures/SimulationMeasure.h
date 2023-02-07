/*
** SimulationMeasure.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Measure.h"

namespace scone
{
	class SimulationMeasure : public Measure
	{
	public:
		SimulationMeasure( const PropNode& pn, Params& par, const Model& model, const Location& loc );

		virtual bool UpdateMeasure( const Model& model, double timestamp ) override;
		virtual double ComputeResult( const Model& model ) override;

	protected:
		virtual String GetClassSignature() const override { return "SIM"; }
	};
}
