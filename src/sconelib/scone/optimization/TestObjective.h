/*
** TestObjective.h
**
** Copyright (C) 2013-2019 Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Objective.h"
#include "xo/utility/smart_enum.h"
#include "spot/spot_types.h"
#include "scone/core/Range.h"
#include "xo/numerical/bounds.h"

namespace scone
{
	/// TestObjective function types
	xo_smart_enum_class( TestObjectiveFunction, Sphere, Ellipsoid, Rosenbrock, Schwefel, Rastrigin );

	/// Objective used for testing
	class SCONE_API TestObjective : public Objective
	{
	public:
		TestObjective( const PropNode& pn, const path& find_file_folder );
		virtual ~TestObjective() {}

		/// Type of the function; default Schwefel
		TestObjectiveFunction function_;

		/// Dimension of the objective function
		size_t dim_;

		/// Parameter value range
		xo::boundsd range_;

		/// Value mean
		Real mean_;

		/// Value std
		Real std_;

		virtual fitness_t evaluate( const SearchPoint& point ) const override;

	protected:
		virtual String GetClassSignature() const override;

		double ( *fun_ )( const spot::par_vec& );
	};
}
