#include "ModelConverter.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/Joint.h"
#include "scone/model/Muscle.h"
#include "scone/model/Dof.h"
#include "xo/numerical/bounds.h"
#include "xo/utility/side.h"
#include "scone/model/model_tools.h"
#include "string_tools.h"
#include "Angle.h"

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

	ModelConverter::ModelConverter( const PropNode& pn ) :
		INIT_MEMBER( pn, joint_stiffness_, 1e6 ),
		INIT_MEMBER( pn, joint_limit_stiffness_, 500 ),
		INIT_MEMBER( pn, body_mass_threshold_, 0.2 )
	{}

	PropNode ModelConverter::ConvertModel( const Model& model )
	{
		PropNode pn;
		auto& model_pn = pn.add_child( "model" );

		ConvertMaterials( model, model_pn );

		for ( auto& b : model.GetBodies() )
			ConvertBody( *b, model_pn );

		for ( auto& m : model.GetMuscles() )
			ConvertMuscle( *m, model_pn );

		for ( auto& cg : model.GetContactGeometries() )
			ConvertContactGeometry( *cg, model_pn );

		for ( auto& d : model.GetDofs() )
			ConvertDof( *d, model_pn );

		return pn;
	}

	void ModelConverter::ConvertBody( const Body& b, PropNode& parent_pn )
	{
		auto& body_pn = parent_pn.add_child( "body" );
		body_pn[ "name" ] = b.GetName();
		body_pn[ "mass" ] = b.GetMass();
		body_pn[ "inertia" ] = b.GetInertiaTensorDiagonal();

		if ( auto* j = b.GetJoint(); j && IsRealJoint( *j ) )
			ConvertJoint( *j, body_pn );
		else if ( !b.GetLocalComPos().is_null() )
			body_pn[ "pos" ] = b.GetLocalComPos();

		for ( const auto& g : b.GetDisplayGeometries() ) {
			auto& geom_pn = body_pn.add_child( "mesh" );
			geom_pn[ "file" ] = g.filename_;
			geom_pn[ "pos" ] = g.pos_ - b.GetLocalComPos();
			if ( g.ori_ != Quat::identity() )
				geom_pn[ "ori" ] = g.ori_;
			if ( g.scale_ != Vec3::diagonal( 1.0 ) )
				geom_pn[ "scale" ] = g.scale_;
		}
	}

	void ModelConverter::ConvertJoint( const Joint& j, PropNode& parent_pn )
	{
		auto& bp = j.GetParentBody();
		auto& bc = j.GetBody();
		auto tfp = xo::transformd{ bp.GetComPos(), bp.GetOrientation() };
		auto tfc = xo::transformd{ bc.GetComPos(), bc.GetOrientation() };
		auto& joint_pn = parent_pn.add_child( "joint" );
		joint_pn[ "name" ] = j.GetName();
		joint_pn[ "parent" ] = j.GetParentBody().GetName();
		joint_pn[ "pos_in_parent" ] = fix( tfp.inv_trans( j.GetPos() ) );
		joint_pn[ "pos_in_child" ] = fix( tfc.inv_trans( j.GetPos() ) );
		auto limits = bounds3rad{ {0,0}, {0,0}, {0,0} };
		for ( auto* dof : j.GetDofs() ) {
			if ( dof->IsRotational() ) {
				auto axis = dof->GetRotationAxis();
				auto range = dof->GetRange();
				limits[ GetAxisIndex( axis ) ] = { Radian( range.min ), Radian( range.max ) };
			}
		}
		joint_pn[ "limits" ] = bounds3deg( limits );
	}

	void ModelConverter::ConvertMuscle( const Muscle& m, PropNode& parent_pn )
	{
		auto& mus_pn = parent_pn.add_child( "point_path_muscle" );
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
	}

	void ModelConverter::ConvertContactGeometry( const ContactGeometry& cg, PropNode& parent_pn )
	{
		auto& cg_pn = parent_pn.add_child( "geometry" );
		cg_pn[ "name" ] = cg.GetName();
		cg_pn.append( to_prop_node( cg.GetShape() ) );
		cg_pn[ "body" ] = cg.GetBody().GetName();
		cg_pn[ "pos" ] = fix( cg.GetPos() - cg.GetBody().GetLocalComPos() );
		cg_pn[ "ori" ] = fix( Vec3( xo::vec3degd( xo::euler_xyz_from_quat( cg.GetOri() ) ) ) );
	}

	void ModelConverter::ConvertDof( const Dof& d, PropNode& parent_pn )
	{
		auto& dof_pn = parent_pn.add_child( "dof" );
		dof_pn[ "name" ] = d.GetName();
		dof_pn[ "source" ] = GetDofSourceName( d );
		auto range = xo::boundsd( d.GetRange().min, d.GetRange().max );
		dof_pn[ "range" ] = d.IsRotational() ? xo::boundsd( boundsdeg( boundsrad( range ) ) ) : range;
		if ( d.GetDefaultPos() != 0.0 )
			dof_pn[ "default" ] = d.IsRotational() ? xo::rad_to_deg( d.GetDefaultPos() ) : d.GetDefaultPos();
	}

	void ModelConverter::ConvertMaterials( const Model& m, PropNode& parent_pn )
	{
		if ( !m.GetContactForces().empty() ) {
			auto& cf = *m.GetContactForces().front();
			auto& mat_pn = parent_pn.add_child( "material" );
			mat_pn[ "name" ] = "default_material";
			mat_pn[ "static_friction" ] = cf.GetStaticFriction();
			mat_pn[ "dynamic_friction" ] = cf.GetDynamicFriction();
			mat_pn[ "stiffness" ] = cf.GetStiffness();
			mat_pn[ "damping" ] = cf.GetDamping();
		}
		auto& opt_pn = parent_pn.add_child( "model_options" );
		opt_pn[ "joint_stiffness" ] = joint_stiffness_;
		opt_pn[ "joint_limit_stiffness" ] = joint_limit_stiffness_;
	}
}

