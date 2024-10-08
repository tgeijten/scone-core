#pragma once
#include "Muscle.h"
#include "xo/container/zip.h"
#include "Side.h"
#include <vector>

namespace scone
{
	template< typename T >
	bool CheckValues( const T& v1, const T& v2, const string& vname, PropNode& pn ) {
		if ( v1 != v2 )
			pn.add_key_value( vname, to_str( v1 ) + " <> " + to_str( v2 ) );
		return v1 == v2;
	}
	bool CheckReal( Real v1, Real v2, const string& vname, PropNode& pn, Real e = 1e-6 ) {
		auto diff = std::abs( v1 - v2 );
		bool equal = diff <= e;
		if ( !equal )
			pn.add_key_value( vname, stringf( "%g (%.8f <> %.8f)", diff, v1, v2 ) );
			//pn.add_key_value( vname, to_str( v1 ) + " <> " + to_str( v2 ) );
		return equal;
	}
	void CheckMirrored( const Vec3& v1, const Vec3& v2, const String& vname, PropNode& pn ) {
		//if ( !IsMirrored( v1, v2, 1e-6 ) )
		//	pn.add_key_value( vname, to_str( v1 ) + " <> " + to_str( v2 ) );
		CheckReal( v1.x, v2.x, vname + ".x", pn );
		CheckReal( v1.y, v2.y, vname + ".y", pn );
		CheckReal( v1.z, -v2.z, vname + ".z", pn );
	}
	PropNode CheckSymmetry( const Muscle& m1, const Muscle& m2 ) {
		PropNode pn;
		auto path1 = m1.GetLocalMusclePath();
		auto path2 = m2.GetLocalMusclePath();
		if ( CheckValues( path1.size(), path2.size(), "path_size", pn ) ) {
			for ( index_t i = 0; i < path1.size(); ++i ) {
				if ( !IsMirrored( path1[i].first->GetName(), path2[i].first->GetName() ) )
					pn.add_value( path1[i].first->GetName() + " <> " + path2[i].first->GetName() );
				CheckMirrored( path1[i].second, path2[i].second, xo::stringf( "path_point[%d]", i ), pn );
			}
		}
		CheckValues( m1.GetMaxIsometricForce(), m2.GetMaxIsometricForce(), "max_isometric_force", pn );
		CheckValues( m1.GetTendonSlackLength(), m2.GetTendonSlackLength(), "tendon_slack_length", pn );
		CheckValues( m1.GetOptimalFiberLength(), m2.GetOptimalFiberLength(), "optimal_fiber_length", pn );
		CheckValues( m1.GetPennationAngleAtOptimal(), m2.GetPennationAngleAtOptimal(), "pennation_angle", pn );
		return pn;
	}

	PropNode CheckSymmetry( const Dof& d1, const Dof& d2 ) {
		PropNode pn;
		CheckValues( d1.GetRange().min, d2.GetRange().min, "range_min", pn );
		CheckValues( d1.GetRange().max, d2.GetRange().max, "range_max", pn );
		return pn;
	}

	template< typename T > void CheckSymmetry( const T& obj, const std::vector<T*>& cont, PropNode& pn ) {
		if ( GetSideFromName( obj.GetName() ) == Side::Left ) {
			auto mir_obj_it = TryFindByName( cont, GetMirroredName( obj.GetName() ) );
			if ( mir_obj_it != cont.end() ) {
				auto& mir_obj = **mir_obj_it;
				auto opn = CheckSymmetry( obj, mir_obj );
				if ( !opn.empty() )
					pn.add_child( obj.GetName() + " <> " + mir_obj.GetName(), opn );
			}
		}
	}

	PropNode CheckSymmetry( const Model& model ) {
		PropNode pn;
		for ( const auto& mus : model.GetMuscles() )
			CheckSymmetry( *mus, model.GetMuscles(), pn );
		for ( const auto& dof : model.GetDofs() )
			CheckSymmetry( *dof, model.GetDofs(), pn );
		return pn;
	}
}

