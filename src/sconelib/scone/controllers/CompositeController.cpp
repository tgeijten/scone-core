/*
** CompositeController.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "CompositeController.h"

#include "scone/core/Factories.h"
#include "xo/container/container_tools.h"
#include "xo/string/string_tools.h"
#include "scone/core/Log.h"

namespace scone
{
	CompositeController::CompositeController( const PropNode& props, Params& par, Model& model, const Location& loc ) :
		Controller( props, par, model, loc )
	{
		for ( auto& cpn : props )
			if ( auto fp = MakeFactoryProps( GetControllerFactory(), cpn, "Controller" ) )
				controllers_.emplace_back( CreateController( fp, par, model, loc ) );

		if ( Controllers = props.try_get_child( "Controllers" ) )
			for ( auto& cpn : *Controllers )
				if ( auto fp = MakeFactoryProps( GetControllerFactory(), cpn, "Controller" ) )
					controllers_.emplace_back( CreateController( fp, par, model, loc ) );

		// show error if child controllers have identical names
		if ( controllers_.size() > 1 )
		{
			for ( auto it1 = controllers_.begin(); it1 != controllers_.end(); ++it1 )
				for ( auto it2 = it1 + 1; it2 != controllers_.end(); ++it2 )
				{
					if ( ( *it1 )->GetName().empty() && ( *it2 )->GetName().empty() )
						log::debug( "Multiple child controllers have no name and may use the same parameters" );
					else if ( ( *it1 )->GetName() == ( *it2 )->GetName() )
						log::warning( "Multiple child controllers are named " + xo::quoted( ( *it1 )->GetName() ) + " and may use the same parameters" );
				}
		}
	}

	bool CompositeController::ComputeControls( Model& model, double timestamp )
	{
		bool terminate = false;
		for ( auto& c : controllers_ )
			terminate |= c->UpdateControls( model, timestamp );
		return terminate;
	}

	bool CompositeController::PerformAnalysis( const Model& model, double timestamp )
	{
		bool terminate = false;
		for ( auto& c : controllers_ )
			terminate |= c->UpdateAnalysis( model, timestamp );
		return terminate;
	}

	void CompositeController::Reset( Model& model )
	{
		for ( auto& c : controllers_ )
			c->Reset( model );
	}

	void CompositeController::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		for ( auto& c : controllers_ )
			c->StoreData( frame, flags );
	}

	std::vector<xo::path> CompositeController::WriteResults( const xo::path& file ) const
	{
		std::vector<xo::path> files;
		for ( auto& c : controllers_ )
			xo::append( files, c->WriteResults( file ) );
		return files;
	}

	PropNode CompositeController::GetInfo() const
	{
		PropNode pn;
		for ( auto& c : controllers_ )
		{
			if ( auto cpn = c->GetInfo(); !cpn.empty() )
				pn.add_child( xo::get_clean_type_name( *c ), cpn );
		}
		return pn;
	}

	String CompositeController::GetClassSignature() const
	{
		std::vector< String > strset;
		for ( auto& c : controllers_ )
		{
			string s = c->GetSignature();
			if ( xo::find( strset, s ) == strset.end() )
				strset.emplace_back( s );
		}
		return xo::concatenate_str( strset, "." );
	}
}
