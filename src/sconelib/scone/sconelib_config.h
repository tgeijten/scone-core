/*
** sconelib_config.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#ifdef SCONE_OPENSIM_3
#	include "sconeopensim3/sconeopensim3.h"
#	define SCONE_OPENSIM_3_ENABLED 1
#else
#	define SCONE_OPENSIM_3_ENABLED 0
#endif

#ifdef SCONE_OPENSIM_4
#	include "sconeopensim4/sconeopensim4.h"
#	define SCONE_OPENSIM_4_ENABLED 1
#else
#	define SCONE_OPENSIM_4_ENABLED 0
#endif

#ifdef SCONE_HYFYDY
#	include "sconehfd/sconehfd_f32.h"
#	define SCONE_HYFYDY_ENABLED 1
#	ifdef SCONE_HYFYDY_DBL 
#		include "sconehfd/sconehfd_f64.h"
#	endif
	namespace sconehfd = scone::sconehfd_f32; // namespace alias for sconestudio, sconecmd, etc.
#else
#	define SCONE_HYFYDY_ENABLED 0
#endif

#ifdef SCONE_LUA
#	include "sconelua/sconelua.h"
#	define SCONE_LUA_ENABLED 1
#else
#	define SCONE_LUA_ENABLED 0
#endif

#ifdef SCONE_USER_EXTENSIONS
#	include "sconeuser/sconeuser.h"
#	define SCONE_USER_EXTENSIONS_ENABLED 1
#else
#	define SCONE_USER_EXTENSIONS_ENABLED 0
#endif

#include "xo/serialization/serialize.h"
#include "xo/serialization/prop_node_serializer_zml.h"

namespace scone
{
	inline void Initialize()
	{
		// register .scone file format
		xo::register_file_extension< xo::prop_node_serializer_zml >( "scone" );

#if SCONE_OPENSIM_3_ENABLED
		RegisterSconeOpenSim3();
#endif

#if SCONE_OPENSIM_4_ENABLED
		RegisterSconeOpenSim4();
#endif

#if SCONE_HYFYDY_ENABLED
		xo::register_file_extension< xo::prop_node_serializer_zml >( "hfd" );
		sconehfd_f32::TryRegisterSconeHfd();
#	ifdef SCONE_HYFYDY_DBL 
		sconehfd_f64::TryRegisterSconeHfd();
#	endif
#endif

#if SCONE_LUA_ENABLED
		RegisterSconeLua();
#endif

#if SCONE_USER_EXTENSIONS_ENABLED
		RegisterSconeUserExtensions();
#endif
	}
}
