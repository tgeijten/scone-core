/*
** platform.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#if defined(_MSC_VER)
#	ifdef SCONE_MODEL_OPENSIM_4_EXPORTS
#		define SCONE_OPENSIM_4_API __declspec(dllexport)
#	else
#		define SCONE_OPENSIM_4_API __declspec(dllimport)
#	endif
#else
#	define SCONE_OPENSIM_4_API
#endif
