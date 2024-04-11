/*
** ConstantFunction.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Function.h"
#include "PropNode.h"
#include "scone/optimization/Params.h"

namespace scone
{
	/// Parameterizable piece-wise constant function.
	class SCONE_API ConstantFunction : public Function
	{
	public:
		ConstantFunction( const PropNode& props, Params& par );
		virtual ~ConstantFunction() = default;

		/// Constant value.
		const Real value;

		virtual Real GetValue( Real x ) override { return value; }

		// a signature describing the function
		virtual String GetSignature() override;
	};
}
