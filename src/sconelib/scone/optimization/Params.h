/*
** Params.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "spot/search_point.h"
#include "spot/objective_info.h"
#include "spot/par_tools.h"
#include "spot/par_info.h"
#include "xo/utility/optional.h"

namespace scone
{
	using Params = spot::par_io;
	using SearchPoint = spot::search_point;
	using ObjectiveInfo = spot::objective_info;
	using ParInfo = spot::par_info;
	using ParValue = spot::par_t;
	using OptionalPar = xo::optional<spot::par_t>;
	using ScopedParamSetPrefixer = spot::scoped_prefix;
}
