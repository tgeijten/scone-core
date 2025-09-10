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
		Controller( props, par, model, loc ),
		INIT_MEMBER( props, dual_sided, false ),
		INIT_MEMBER( props, symmetric, loc.symmetric_ ),
		INIT_MEMBER( props, child_names, std::vector<String>() )
	{
		auto child_count = child_names.size();
		
		do {
			// directly added controllers
			for ( auto& cpn : props ) {
				if ( child_count > 0 && controllers_.size() == child_count )
					break;
				if ( auto fp = MakeFactoryProps( GetControllerFactory(), cpn, "Controller" ) ) {
					ScopedParamSetPrefixer pp( par, child_count > 0 ? child_names[controllers_.size() % child_count] : "" );
					CreateChildController( fp, par, model, loc );
				}
			}

			// controllers in Controllers section (old-style)
			if ( Controllers = props.try_get_child( "Controllers" ) ) {
				if ( child_count > 0 && controllers_.size() == child_count )
					break;
				for ( auto& cpn : *Controllers )
					if ( auto fp = MakeFactoryProps( GetControllerFactory(), cpn, "Controller" ) ) {
						ScopedParamSetPrefixer pp( par, child_count > 0 ? child_names[controllers_.size() % child_count] : "" );
						CreateChildController( fp, par, model, loc );
					}
			}
		} while ( controllers_.size() > 0 && controllers_.size() < child_count );

		// show error if child controllers have identical names
		if ( controllers_.size() > 1 && child_names.empty() ) {
			for ( auto it1 = controllers_.begin(); it1 != controllers_.end(); ++it1 )
				for ( auto it2 = it1 + 1; it2 != controllers_.end(); ++it2 ) {
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

	UpdateResult CompositeController::PerformAnalysis( const Model& model, double timestamp )
	{
		UpdateResult result;
		for ( auto& c : controllers_ )
			result |= c->UpdateAnalysis( model, timestamp );
		return result;
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

	Controller* CompositeController::InsertChildController( ControllerUP child, index_t pos )
	{
		auto it = controllers_.begin() + std::min( pos, controllers_.size() );
		return &**controllers_.insert( it, std::move( child ) );
	}

	int CompositeController::TrySetControlParameter( const String& name, Real value )
	{
		int result = 0;
		for ( auto& c : controllers_ )
			result += c->TrySetControlParameter( name, value );
		return result;
	}

	xo::optional<Real> CompositeController::TryGetControlParameter( const String& name )
	{
		for ( auto& c : controllers_ )
			if ( auto v = c->TryGetControlParameter( name ) )
				return *v;
		return {};
	}

	std::vector<String> CompositeController::GetControlParameters() const
	{
		std::vector<String> results;
		for ( auto& c : controllers_ )
			xo::append( results, c->GetControlParameters() );
		return results;
	}

	String CompositeController::GetClassSignature() const
	{
		std::vector< String > strset;
		for ( auto& c : controllers_ ) {
			string s = c->GetSignature();
			if ( xo::find( strset, s ) == strset.end() )
				strset.emplace_back( s );
		}
		return xo::concat_str( strset, "." );
	}

	void CompositeController::CreateChildController( const FactoryProps& fp, Params& par, Model& model, const Location& loc )
	{
		if ( dual_sided ) {
			for ( auto s : LeftAndRightSide )
				controllers_.emplace_back( CreateController( fp, par, model, Location{ s, symmetric } ) );
		}
		else controllers_.emplace_back( CreateController( fp, par, model, Location{ loc.side_, symmetric } ) );
	}
}
