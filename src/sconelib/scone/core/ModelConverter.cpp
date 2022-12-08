#include "ModelConverter.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/Joint.h"
#include "scone/model/Muscle.h"
#include "scone/model/Dof.h"
#include "xo/numerical/bounds.h"
#include "xo/utility/side.h"

namespace scone
{
	using boundsrad = xo::bounds< xo::angle_< xo::angle_unit::radians, double > >;
	using boundsdeg = xo::bounds< xo::angle_< xo::angle_unit::degrees, double > >;
	using bounds3rad = xo::vec3_< boundsrad >;
	using bounds3deg = xo::vec3_< boundsdeg >;
	using radian = xo::radiand;
	using degree = xo::degreed;

	static constexpr Real fix_epsilon = 1e-6;
	inline Real fix( Real v ) { return std::abs( v ) < fix_epsilon ? Real( 0 ) : v; }
	inline Vec3 fix( Vec3 v ) { return Vec3( fix( v.x ), fix( v.y ), fix( v.z ) ); }

	bool IsRealJoint( const Joint& j ) {
		return j.GetParentBody().GetMass() > 0;
	}
	struct dof_info {
		string name;
		bool mirror_left = false;
	};
	string GetDofSourceName( const Dof& dof ) {
		static xo::flat_map<string, dof_info> dof_types{
			{ "tx", { "tx" } },
			{ "ty", { "ty" } },
			{ "tz", { "tz" } },
			{ "tilt", { "rz" } },
			{ "extension", { "rz" } },
			{ "list", { "rx" } },
			{ "bending", { "rx" } },
			{ "rotation", { "ry", true } },
			{ "flexion", { "rz" } },
			{ "angle", { "rz" } },
			{ "adduction", { "rx", true } },
			{ "inversion", { "rx", true } }
		};

		const auto& j = *dof.GetJoint();
		auto split_name = xo::split_str( dof.GetName(), "_" );
		if ( split_name.size() >= 2 )
		{
			string& dof_base_name = split_name[ 0 ];
			string& dof_type_name = split_name[ 1 ];
			auto side = xo::str_get_side( j.GetName() );
			auto base_name = xo::str_remove_side( IsRealJoint( j ) ? j.GetName() : j.GetBody().GetName() );
			if ( auto di = dof_types.find( dof_type_name ); di != dof_types.end() )
			{
				string prefix = di->second.mirror_left && side == xo::side::left ? "-" : "";
				return prefix + base_name + "_" + di->second.name + xo::side_postfix( side );
			}
		}

		xo::log::warning( "Cannot deduce dof name, coordinate=", dof.GetJoint(), " joint=", j.GetName() );
		return "?";
	}

	ModelConverter::ModelConverter( const PropNode& pn ) :
		INIT_MEMBER( pn, joint_stiffness_, 1e6 ),
		INIT_MEMBER( pn, joint_limit_stiffness_, 500 ),
		INIT_MEMBER( pn, body_mass_threshold_, 0.2 )
	{}

	PropNode ModelConverter::ConvertModel( const Model& model )
	{
		PropNode pn;
		auto& model_pn = pn.add_child( "model" );

		for ( auto& b : model.GetBodies() )
			model_pn.append( ConvertBody( *b ) );

		for ( auto& m : model.GetMuscles() )
			model_pn.append( ConvertMuscle( *m ) );

		for ( auto& cg : model.GetContactGeometries() )
			model_pn.append( ConvertContactGeometry( *cg ) );

		for ( auto& d : model.GetDofs() )
			model_pn.append( ConvertDof( *d ) );

		return pn;
	}

	PropNode ModelConverter::ConvertBody( const Body& b )
	{
		PropNode pn;
		auto& body_pn = pn.add_child( "body" );
		body_pn[ "name" ] = b.GetName();
		body_pn[ "mass" ] = b.GetMass();
		body_pn[ "inertia" ] = b.GetInertiaTensorDiagonal();

		if ( auto* j = b.GetJoint() ) {
			auto& bp = j->GetParentBody();
			auto& bc = j->GetBody();
			auto tfp = xo::transformd{ bp.GetComPos(), bp.GetOrientation() };
			auto tfc = xo::transformd{ bc.GetComPos(), bc.GetOrientation() };

			if ( bp.GetMass() > 0 ) {
				auto& joint_pn = body_pn.add_child( "joint" );
				joint_pn[ "name" ] = j->GetName();
				joint_pn[ "parent" ] = j->GetParentBody().GetName();
				joint_pn[ "pos_in_parent" ] = fix( tfp.inv_trans( j->GetPos() ) );
				joint_pn[ "pos_in_child" ] = fix( tfc.inv_trans( j->GetPos() ) );
			}
			else {
				body_pn[ "pos" ] = bc.GetLocalComPos();
			}
		}

		for ( const auto& g : b.GetDisplayGeometries() ) {
			auto& geom_pn = body_pn.add_child( "mesh" );
			geom_pn[ "file" ] = g.filename_;
			geom_pn[ "pos" ] = g.pos_ - b.GetLocalComPos();
			if ( g.ori_ != Quat::identity() )
				geom_pn[ "ori" ] = g.ori_;
			if ( g.scale_ != Vec3::diagonal( 1.0 ) )
				geom_pn[ "scale" ] = g.scale_;
		}

		return pn;
	}

	PropNode ModelConverter::ConvertMuscle( const Muscle& m )
	{
		PropNode pn;
		auto& mus_pn = pn.add_child( "point_path_muscle" );
		mus_pn[ "name" ] = m.GetName();
		mus_pn[ "max_isometric_force" ] = m.GetMaxIsometricForce();
		mus_pn[ "optimal_fiber_length" ] = m.GetOptimalFiberLength();
		mus_pn[ "tendon_slack_length" ] = m.GetTendonSlackLength();
		mus_pn[ "pennation_angle" ] = m.GetPennationAngleAtOptimal();
		auto& path_pn = mus_pn.add_child( "path" );
		auto mp = m.GetLocalMusclePath();
		for ( auto& [body, point] : mp ) {
			auto& ppn = path_pn.add_child();
			ppn[ "body" ] = body->GetName();
			ppn[ "pos" ] = point - body->GetLocalComPos();
		}
		return pn;
	}

	PropNode ModelConverter::ConvertContactGeometry( const ContactGeometry& cg )
	{
		PropNode pn;
		auto& cg_pn = pn.add_child( "geometry" );
		cg_pn[ "name" ] = cg.GetName();
		cg_pn.append( to_prop_node( cg.GetShape() ) );
		cg_pn[ "body" ] = cg.GetBody().GetName();
		cg_pn[ "pos" ] = fix( cg.GetPos() - cg.GetBody().GetLocalComPos() );
		cg_pn[ "ori" ] = fix( Vec3( xo::vec3degd( xo::euler_xyz_from_quat( cg.GetOri() ) ) ) );
		return pn;
	}

	PropNode ModelConverter::ConvertDof( const Dof& d )
	{
		PropNode pn;
		auto& dof_pn = pn.add_child( "dof" );
		dof_pn[ "name" ] = d.GetName();
		dof_pn[ "source" ] = GetDofSourceName( d );
		auto range = xo::boundsd( d.GetRange().min, d.GetRange().max );
		dof_pn[ "range" ] = d.IsRotational() ? xo::boundsd( boundsdeg( boundsrad( range ) ) ) : range;
		if ( d.GetDefaultPos() != 0.0 )
			dof_pn[ "default" ] = d.IsRotational() ? xo::rad_to_deg( d.GetDefaultPos() ) : d.GetDefaultPos();
		return pn;
	}
}
