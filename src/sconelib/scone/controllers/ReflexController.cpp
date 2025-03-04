/*
** ReflexController.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "ReflexController.h"

#include "scone/core/Log.h"
#include "scone/core/profiler_config.h"
#include "scone/core/Factories.h"

#include "scone/model/Model.h"
#include "scone/model/Muscle.h"
#include "scone/model/Location.h"

#include "MuscleReflex.h"

#include "xo/string/string_tools.h"

namespace scone
{
	ReflexController::ReflexController( const PropNode& props, Params& par, Model& model, const Location& loc ) :
		Controller( props, par, model, loc )
	{
		INIT_PROP( props, symmetric, loc.symmetric_ );
		INIT_PROP( props, dual_sided, loc.side_ == Side::None );

		// create reflexes for single or both sides
		auto create_reflex = [&]( const FactoryProps& fp ) {
			if ( dual_sided ) {
				for ( auto side : { Side::Left, Side::Right } )
					m_Reflexes.push_back( CreateReflex( fp, par, model, *this, Location( side, symmetric ) ) );
			}
			else m_Reflexes.push_back( CreateReflex( fp, par, model, *this, Location( loc.side_, symmetric ) ) );
		};

		for ( const auto& item : props )
			if ( auto fp = MakeFactoryProps( GetReflexFactory(), item, "Reflex" ) )
				create_reflex( fp );

		if ( Reflexes = props.try_get_child( "Reflexes" ) )
			for ( auto& item : *Reflexes )
				if ( auto fp = MakeFactoryProps( GetReflexFactory(), item, "Reflex" ) )
					create_reflex( fp );
	}

	ReflexController::~ReflexController()
	{}

	bool ReflexController::ComputeControls( Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		// IMPORTANT: delayed storage must have been updated in through Model::UpdateSensorDelayAdapters()
		for ( ReflexUP& r : m_Reflexes )
			r->ComputeControls( timestamp );

		return false;
	}

	String ReflexController::GetClassSignature() const
	{
		return "R" + xo::to_str( m_Reflexes.size() );
	}

	void ReflexController::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		for ( auto& r : m_Reflexes )
			r->StoreData( frame, flags );
	}

	int ReflexController::TrySetControlParameter( const String& name, Real value )
	{
		int result = 0;
		for ( auto& r : m_Reflexes )
			result += r->TrySetControlParameter( name, value );
		return result;
	}

	xo::optional<Real> ReflexController::TryGetControlParameter( const String& name )
	{
		for ( auto& r : m_Reflexes )
			if ( auto v = r->TryGetControlParameter( name ) )
				return *v;
		return {};
	}

	std::vector<String> ReflexController::GetControlParameters() const
	{
		std::vector<String> results;
		for ( auto& r : m_Reflexes )
			xo::append( results, r->GetControlParameters() );
		return results;
	}
}
