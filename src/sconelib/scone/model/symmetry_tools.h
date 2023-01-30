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

	bool CheckSymmetry( const Dof& d1, const Dof& d2 ) {
		if ( d1.GetRange().min != d2.GetRange().min )
			return false;
		if ( d1.GetRange().max != d2.GetRange().max )
			return false;
		return true;
	}

	template< typename T > bool CheckSymmetry( const T& obj, const std::vector<T*>& cont ) {
		auto mir_ob = TryFindByName( cont, GetMirroredName( obj.GetName() ) );
		if ( mir_ob == cont.end() )
			return false;
		return CheckSymmetry( obj, **mir_ob );
	}

	std::vector<string> CheckSymmetry( const Model& model ) {
		std::vector<string> objects;
		for ( const auto& mus : model.GetMuscles() ) {
			if ( !CheckSymmetry( *mus, model.GetMuscles() ) )
				objects.push_back( mus->GetName() );
		}
		for ( const auto& dof : model.GetDofs() ) {
			if ( !CheckSymmetry( *dof, model.GetDofs() ) )
				objects.push_back( dof->GetName() );
		}
		return objects;
	}
}
