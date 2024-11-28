/*
** model_tools.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "model_tools.h"

#include "scone/core/string_tools.h"
#include "scone/core/profiler_config.h"
#include "scone/model/UserInput.h"
#include "xo/container/flat_map.h"
#include "xo/utility/side.h"
#include "xo/system/log.h"
#include "Dof.h"
#include "Joint.h"
#include "Body.h"
#include "xo/container/container_tools.h"

using std::vector;
using std::pair;

namespace scone
{
	Vec3 GetGroundCop( const Vec3& force, const Vec3& moment, Real min_force )
	{
		if ( force.y >= min_force )
			return Vec3( moment.z / force.y, 0, -moment.x / force.y );
		else return Vec3::zero();
	}

	Vec3 GetPlaneCop( const Vec3& normal, const Vec3& location, const Vec3& force, const Vec3& moment, Real min_force /*= REAL_WIDE_EPSILON */ )
	{
		if ( dot_product( normal, force ) >= min_force )
		{
			auto normal_force_scalar = xo::dot_product( force, normal );
			auto pos0 = xo::cross_product( normal, moment ) / normal_force_scalar;
			Vec3 delta_pos = pos0 - location;
			double p1 = dot_product( delta_pos, normal );
			double p2 = dot_product( force, normal );
			auto pos = pos0 - ( p1 / p2 ) * force;
			return pos;
		}
		else return Vec3::zero();
	}

	PropNode MakePropNode( const std::vector<UserInputUP>& user_inputs )
	{
		PropNode pn;
		for ( auto& ui : user_inputs )
			pn[ui->GetName()] = ui->GetValue();
		return pn;
	}

	size_t SetUserInputsFromPropNode( const PropNode& pn, const std::vector<UserInputUP>& user_inputs )
	{
		size_t count = 0;
		for ( auto& ui : user_inputs )
		{
			if ( auto v = pn.try_get<Real>( ui->GetName() ) )
				ui->SetValue( *v ), ++count;
		}
		return count;
	}

	bool IsRealJoint( const Joint& j )
	{
		auto trans_dofs = xo::count_if( j.GetDofs(), [&]( auto&& d ) { return !d->IsRotational(); } );
		return j.GetParentBody().GetMass() > 0 || trans_dofs == 0;
	}

	bool IsWeldedBody( const Body& b )
	{
		auto* j = b.GetJoint();
		return j && IsRealJoint( *j ) && j->GetDofs().empty() && b.GetMass() > 0;
	}

	const Body* GetWeldedRoot( const Body& b )
	{
		if ( b.GetMass() == 0 )
			return nullptr;
		if ( auto* j = b.GetJoint() ) {
			if ( !j->GetDofs().empty() )
				return &b;
			else return GetWeldedRoot( j->GetParentBody() );
		}
		else return nullptr;
	}

	struct dof_info {
		string name;
		bool mirror_left = false;
	};

	string GetDofSourceNameLookUp( const Dof& dof ) {
		static xo::flat_map<string, dof_info> dof_types{
			{ "tx",{ "tx" } },
			{ "ty",{ "ty" } },
			{ "tz",{ "tz" } },
			{ "tilt",{ "rz" } },
			{ "extension",{ "rz" } },
			{ "list",{ "rx" } },
			{ "bending",{ "rx" } },
			{ "rotation",{ "ry", true } },
			{ "flexion",{ "rz" } },
			{ "angle",{ "rz" } },
			{ "adduction",{ "rx", true } },
			{ "inversion",{ "rx", true } }
		};

		const auto& j = *dof.GetJoint();
		auto split_name = xo::split_str( dof.GetName(), "_" );
		if ( split_name.size() >= 2 )
		{
			string& dof_base_name = split_name[0];
			string& dof_type_name = split_name[1];
			auto side = xo::str_get_side( j.GetName() );
			auto base_name = xo::str_remove_side( IsRealJoint( j ) ? j.GetName() : j.GetBody().GetName() );
			if ( auto di = dof_types.find( dof_type_name ); di != dof_types.end() )
			{
				string prefix = di->second.mirror_left && side == xo::side::left ? "-" : "";
				return prefix + base_name + "_" + di->second.name + xo::side_postfix( side );
			}
		}

		xo::log::warning( "Cannot deduce dof name, coordinate=", dof.GetName(), " joint=", j.GetName() );
		return "?";
	}

	string GetDofSourceName( const Dof& dof, bool enable_pin_joint )
	{
		static const char* rot_postfix[] = { "_rx", "_ry", "_rz", "_r" };
		static const char* trans_postfix[] = { "_tx", "_ty", "_tz", "_t" };
		const auto* j = dof.GetJoint();
		bool pin_joint = enable_pin_joint && j && j->GetDofs().size() == 1;

		bool negative = dof.IsRotational() && !pin_joint && GetDominantComponentSign( dof.GetLocalAxis() ) < 0;
		String name = negative ? "-" : "";

		if ( j )
			name += xo::str_remove_side( IsRealJoint( *j ) ? j->GetName() : j->GetBody().GetName() );
		else name += "unknown"; // #todo: somehow add body name here

		auto idx = pin_joint ? 3 : GetDominantComponentIndex( dof.GetLocalAxis() );
		name += dof.IsRotational() ? rot_postfix[idx] : trans_postfix[idx];

		auto side = j ? xo::str_get_side( j->GetName() ) : xo::side::none;
		name += xo::side_postfix( side );

		return name;
	}

	const char* GetAxisName( index_t axis )
	{
		static const char* axis_names[] = { "X", "Y", "Z" };
		return axis < 3 ? axis_names[axis] : "";
	}

	const char* GetDominantComponentName( const Vec3& dir )
	{
		return GetAxisName( GetDominantComponentIndex( dir ) );
	}

	string GetVectorIdentifier( const Vec3& dir )
	{
		if ( auto idx = GetAxisIndex( dir ); idx != no_index )
			return GetAxisName( idx ); // a pure component name, hurray
		else return stringf( "%f_%f_%f", dir.x, dir.y, dir.z ); // no pure component
	}

	index_t GetAxisIndex( const Vec3& v )
	{
		if ( v == Vec3::unit_x() ) return 0;
		else if ( v == Vec3::unit_y() ) return 1;
		else if ( v == Vec3::unit_z() ) return 2;
		else return no_index;
	}

	index_t GetDominantComponentIndex( const Vec3& dir )
	{
		auto x = std::abs( dir.x ), y = std::abs( dir.y ), z = std::abs( dir.z );
		if ( x > y && x > z )
			return 0;
		else if ( y > z )
			return 1;
		else return 2;
	}

	Real GetDominantComponentSign( const Vec3& dir )
	{
		return dir[GetDominantComponentIndex( dir )] < 0 ? -1 : 1;
	}
}
