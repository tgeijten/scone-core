#pragma once
#include "Muscle.h"
#include "xo/container/zip.h"
#include "Side.h"
#include <vector>

namespace scone
{
	bool CheckSymmetry( const Muscle& m1, const Muscle& m2 ) {
		auto path1 = m1.GetLocalMusclePath();
		auto path2 = m2.GetLocalMusclePath();
		if ( path1.size() != path2.size() )
			return false;
		for ( auto&& [p1, p2] : xo::zip( path1, path2 ) ) {
			if ( !IsMirrored( p1.first->GetName(), p2.first->GetName() ) )
				return false;
			if ( !IsMirrored( p1.second, p2.second ) )
				return false;
		}
		if ( m1.GetMaxIsometricForce() != m2.GetMaxIsometricForce() )
			return false;
		if ( m1.GetTendonSlackLength() != m2.GetTendonSlackLength() )
			return false;
		if ( m1.GetOptimalFiberLength() != m2.GetOptimalFiberLength() )
			return false;
		if ( m1.GetPennationAngleAtOptimal() != m2.GetPennationAngleAtOptimal() )
			return false;
		return true;
	}

	std::vector<string> CheckSymmetry( const Model& model ) {
		std::vector<string> objects;
		for ( const auto& mus : model.GetMuscles() ) {
			const auto& om = FindByName( model.GetMuscles(), GetMirroredName( mus->GetName() ) );
			if ( !CheckSymmetry( *mus, *om ) ) {
				objects.push_back( mus->GetName() );
				log::warning( "Muscles ", mus->GetName(), " and ", om->GetName(), " are not symmetric" );
			}
		}
		return objects;
	}
}
