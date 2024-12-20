/*
** NoiseController.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "ScriptController.h"
#include "scone/model/Model.h"
#include "scone/model/Actuator.h"
#include "scone/core/string_tools.h"
#include "lua_script.h"
#include "lua_api.h"
#include "scone/core/Log.h"
#include "scone/core/profiler_config.h"
#include "xo/container/prop_node_tools.h"

namespace scone
{
	ScriptController::ScriptController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		CompositeController( pn, par, model, loc ),
		script_file( FindFile( pn.get<path>( "script_file" ) ) ),
		INIT_MEMBER( pn, external_files, std::vector<path>() ),
		script_( new lua_script( script_file, pn ) )
	{
		// optional functions
		if ( auto f = script_->try_find_function( "init" ) )
			init_ = f;
		if ( auto f = script_->try_find_function( "store_data" ) )
			store_ = f;

		// update is required
		update_ = script_->find_function( "update" );

		if ( init_ )
		{
			LuaParams lp( par );
			LuaModel lm( model );
			LuaController lc( *this );
			auto side = static_cast<double>( loc.GetSide() );
			init_( &lm, &lp, side, &lc );
		}

		// add lua files as external resources
		model.AddExternalResource( script_->script_file_ );
		for ( auto& f : external_files )
			model.AddExternalResource( FindFile( f ) );
	}

	ScriptController::~ScriptController()
	{}

	void ScriptController::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		if ( store_ )
		{
			LuaFrame lf( frame );
			store_( &lf );
		}

		CompositeController::StoreData( frame, flags );
	}

	int ScriptController::TrySetControlParameter( const String& name, Real value )
	{
		int result = 0;
		if ( auto it = control_parameters_.find( name ); it != control_parameters_.end() ) {
			it->second = value;
			result++;
		}
		result += CompositeController::TrySetControlParameter( name, value );
		return result;
	}

	xo::optional<Real> ScriptController::TryGetControlParameter( const String& name )
	{
		if ( auto it = control_parameters_.find( name ); it != control_parameters_.end() )
			return it->second;
		else return CompositeController::TryGetControlParameter( name );
	}

	std::vector<String> ScriptController::GetControlParameters() const
	{
		std::vector<String> results = CompositeController::GetControlParameters();
		for ( auto& [name, value] : control_parameters_ )
			results.emplace_back( name );
		return results;
	}

	bool ScriptController::CreateControlParameter( const String& name, Real value )
	{
		// #todo: check if it already exists?
		control_parameters_[name] = value;
		return true;
	}

	bool ScriptController::ComputeControls( Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		LuaModel lm( model );
		LuaController lc( *this );
		bool terminate = update_( &lm, timestamp, &lc );
		terminate |= CompositeController::ComputeControls( model, timestamp );
		return terminate;
	}

	String ScriptController::GetClassSignature() const
	{
		return "SC";
	}
}
