#include "ModelConverter.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/Joint.h"
#include "scone/model/Muscle.h"
#include "scone/model/Dof.h"
#include "xo/numerical/bounds.h"
#include "xo/utility/side.h"
#include "xo/utility/optional.h"
#include "scone/model/model_tools.h"
#include "scone/core/Log.h"
#include "string_tools.h"
#include "Angle.h"
#include "scone/model/Ligament.h"

namespace scone
{
	static constexpr Real fix_epsilon = 1e-6;
	inline Real fix( Real v ) { return std::abs( v ) < fix_epsilon ? Real( 0 ) : v; }
	inline Vec3 fix( Vec3 v ) { return Vec3( fix( v.x ), fix( v.y ), fix( v.z ) ); }

	ModelConverter::ModelConverter( const PropNode& pn ) :
		INIT_MEMBER( pn, joint_stiffness_, 1e6 ),
		INIT_MEMBER( pn, joint_limit_stiffness_, 500 ),
		INIT_MEMBER( pn, body_mass_threshold_, 0.1 )
	{}

	PropNode ModelConverter::ConvertModel( Model& model )
	{
		// Set coordinates to 0 for correct joint orientations
		model.SetNullState();

		PropNode pn;
		auto& model_pn = pn.add_child( "model" );

		ConvertMaterials( model, model_pn );

		for ( auto& b : model.GetBodies() )
			MakeBodyInfo( *b );

		StringSet converted_bodies;
		for ( auto& b : model.GetBodies() )
			ConvertBody( *b, model_pn, converted_bodies );

		for ( auto& m : model.GetMuscles() )
			ConvertMuscle( *m, model_pn );

		if ( convert_ligaments ) {
			for ( auto& m : model.GetLigaments() )
				ConvertLigament( *m, model_pn );
		}

		for ( auto& cg : model.GetContactGeometries() )
			ConvertContactGeometry( *cg, model_pn );

		for ( auto& d : model.GetDofs() )
			ConvertDof( *d, model_pn );

		// Restore original position
		model.Reset();

		return pn;
	}

	void ModelConverter::ConvertBody( const Body& b, PropNode& model_pn, StringSet& converted ) const
	{
		if ( converted.contains( b.GetName() ) )
			return;

		// check if parent body is converted
		if ( auto* pb = b.GetParentBody() )
			ConvertBody( *pb, model_pn, converted );

		// check if this a weld body that should be compounded to its parent
		if ( IsCompoundChild( b ) ) {
			auto& pb = GetCompoundRoot( b );
			auto& body_pn = FindBodyPropNode( pb, model_pn );
			ConvertDisplayGeometry( b, body_pn );
		}
		else {
			auto& body_pn = model_pn.add_child( "body" );
			const auto& gb = GetGlobalBody( b );
			body_pn["name"] = gb.name;
			body_pn["mass"] = gb.mass;
			body_pn["inertia"] = gb.inertia;
			if ( keep_body_origin_ && !b.GetLocalComPos().is_null() )
				body_pn["com_pos"] = b.GetLocalComPos();
			if ( use_body_mass_threshold_ && gb.mass > 0 && gb.mass < body_mass_threshold_ )
				log::warning( gb.name, " mass is below threshold (", gb.mass, " < ", body_mass_threshold_, ")" );

			// joint
			auto* j = b.GetJoint();
			if ( j && IsRealJoint( *j ) )
				ConvertJoint( *j, body_pn );
			else if ( !b.GetLocalComPos().is_null() ) {
				// root body, add pos and ori
				body_pn["pos"] = b.GetLocalComPos();
				if ( !b.GetOrientation().is_identity() )
					body_pn["ori"] = b.GetOrientation();
			}
			ConvertDisplayGeometry( b, body_pn );
		}

		converted.insert( b.GetName() );
	}

	void ModelConverter::ConvertJoint( const Joint& j, PropNode& body_pn ) const
	{
		bool pin_joint = use_pint_joints_ && j.GetDofs().size() == 1;
		auto& bp = j.GetParentBody();
		auto& bc = j.GetBody();
		auto& joint_pn = body_pn.add_child( pin_joint ? "pin_joint" : "joint" );
		joint_pn["name"] = j.GetName();
		joint_pn["parent"] = j.GetParentBody().GetName();
		joint_pn["pos_in_parent"] = fix( GetLocalBodyPos( j.GetPosInParent(), bp ) );
		joint_pn["pos_in_child"] = fix( GetLocalBodyPos( j.GetPosInChild(), bc ) );
		auto ref_ori = xo::quat_from_quats( bp.GetOrientation(), bc.GetOrientation() );
		if ( !xo::is_identity( ref_ori, 1e-9 ) )
			joint_pn["ref_ori"] = ref_ori;
		auto limits = Bounds3Deg{ {0,0}, {0,0}, {0,0} };
		for ( auto* dof : j.GetDofs() ) {
			if ( dof->IsRotational() ) {
				auto axis = dof->GetLocalAxis();
				index_t axis_idx = pin_joint ? 0 : GetDominantComponentIndex( axis );
				double sign = pin_joint ? 1 : GetDominantComponentSign( axis );
				auto dof_info = dof->GetInfo();
				if ( pin_joint )
					joint_pn["axis"] = axis;
				if ( dof_info.has_key( "limit_stiffness" ) ) {
					// convert stiffness and damping
					if ( use_stiffness_from_limit_force_ ) {
						auto kp = dof_info.get<Real>( "limit_stiffness" );
						auto kd = dof_info.get<Real>( "limit_damping" );
						joint_pn["limit_stiffness"] = kp * 180 / xo::num<Real>::pi;
						joint_pn["limit_damping"] = kd / ( kp * joint_limit_damping_angle_ ) * 180 / xo::num<Real>::pi;
					}
					// set limit range
					auto dof_range = dof_info.get<BoundsDeg>( "limit_range" );
					limits[axis_idx] = sign >= 0 ? dof_range : -dof_range;
				}
				else {
					if ( use_limits_from_dof_range_ ) {
						auto dof_range = BoundsRad( dof->GetRange().min, dof->GetRange().max );
						limits[axis_idx] = sign >= 0 ? dof_range : -dof_range;
					}
					else limits[axis_idx] = { Degree( -360 ), Degree( 360 ) };
				}
			}
		}
		if ( pin_joint )
			joint_pn["limits"] = limits[0];
		else joint_pn["limits"] = limits;
	}

	void ModelConverter::ConvertMuscle( const Muscle& m, PropNode& parent_pn ) const
	{
		auto& mus_pn = parent_pn.add_child( "point_path_muscle" );
		mus_pn["name"] = m.GetName();
		mus_pn["max_isometric_force"] = m.GetMaxIsometricForce();
		mus_pn["optimal_fiber_length"] = m.GetOptimalFiberLength();
		mus_pn["tendon_slack_length"] = m.GetTendonSlackLength();
		mus_pn["pennation_angle"] = m.GetPennationAngleAtOptimal();
		auto& path_pn = mus_pn.add_child( "path" );
		auto mp = m.GetLocalMusclePath();
		for ( auto& [b, point] : mp ) {
			auto& ppn = path_pn.add_child();
			ppn["body"] = GetBodyName( *b );
			ppn["pos"] = GetLocalBodyPos( point, *b );
		}
	}

	void ModelConverter::ConvertLigament( const Ligament& m, PropNode& parent_pn ) const
	{
		auto& mus_pn = parent_pn.add_child( "point_path_ligament" );
		mus_pn["name"] = m.GetName();
		mus_pn["force_multiplier"] = m.GetPcsaForce();
		mus_pn["resting_length"] = m.GetRestingLength();
		auto& path_pn = mus_pn.add_child( "path" );
		auto mp = m.GetLocalLigamentPath();
		for ( auto& [b, point] : mp ) {
			auto& ppn = path_pn.add_child();
			ppn["body"] = GetBodyName( *b );
			ppn["pos"] = GetLocalBodyPos( point, *b );
		}
	}

	void ModelConverter::ConvertContactGeometry( const ContactGeometry& cg, PropNode& body_pn ) const
	{
		auto& cg_pn = body_pn.add_child( "geometry" );
		cg_pn["name"] = cg.GetName();
		cg_pn.append( to_prop_node( cg.GetShape() ) );
		cg_pn["body"] = GetBodyName( cg.GetBody() );
		cg_pn["pos"] = GetLocalBodyPos( cg.GetPos(), cg.GetBody() );
		cg_pn["ori"] = fix( Vec3( xo::vec3degd( xo::euler_xyz_from_quat( cg.GetOri() ) ) ) );
	}

	void ModelConverter::ConvertDisplayGeometry( const Body& b, PropNode& body_pn ) const
	{
		for ( const auto& g : b.GetDisplayGeometries() ) {
			auto& geom_pn = body_pn.add_child( "mesh" );
			geom_pn["file"] = g.filename_;
			geom_pn["pos"] = fix( GetLocalBodyPos( g.pos_, b ) );
			if ( g.ori_ != Quat::identity() )
				geom_pn["ori"] = g.ori_;
			if ( g.scale_ != Vec3::one() )
				geom_pn["scale"] = g.scale_;
		}
	}

	void ModelConverter::ConvertDof( const Dof& d, PropNode& parent_pn ) const
	{
		auto& dof_pn = parent_pn.add_child( "dof" );
		dof_pn["name"] = d.GetName();
		dof_pn["source"] = GetDofSourceName( d, use_pint_joints_ );
		auto range = xo::boundsd( d.GetRange().min, d.GetRange().max );
		dof_pn["range"] = d.IsRotational() ? xo::boundsd( BoundsDeg( BoundsRad( range ) ) ) : range;
		if ( d.GetDefaultPos() != 0.0 )
			dof_pn["default"] = d.IsRotational() ? xo::rad_to_deg( d.GetDefaultPos() ) : d.GetDefaultPos();
	}

	void ModelConverter::ConvertMaterials( const Model& m, PropNode& parent_pn ) const
	{
		if ( !m.GetContactForces().empty() ) {
			auto& cf = *m.GetContactForces().front();
			auto& mat_pn = parent_pn.add_child( "material" );
			mat_pn["name"] = "default_material";
			mat_pn["static_friction"] = cf.GetStaticFriction();
			mat_pn["dynamic_friction"] = cf.GetDynamicFriction();
			mat_pn["stiffness"] = cf.GetStiffness();
			mat_pn["damping"] = cf.GetDamping();
		}
		auto& opt_pn = parent_pn.add_child( "model_options" );
		opt_pn["joint_stiffness"] = joint_stiffness_;
		opt_pn["joint_limit_stiffness"] = joint_limit_stiffness_;
	}

	PropNode& ModelConverter::FindBodyPropNode( const Body& b, PropNode& model_pn ) const
	{
		for ( auto& [key, value] : model_pn )
			if ( key == "body" && value.try_get<std::string>( "name" ) == b.GetName() )
				return value;
		SCONE_ERROR( "Could not find node for body " + b.GetName() );
	}

	const ModelConverter::BodyInfo& ModelConverter::GetGlobalBody( const Body& b ) const
	{
		return global_bodies_[GetCompoundRoot( b ).GetName()];
	}

	Vec3 ModelConverter::GetLocalBodyPos( const Vec3& osim_pos, const Body& b ) const
	{
		if ( !keep_body_origin_ ) {
			auto& bi = GetGlobalBody( b );
			return conjugate( bi.ori_world ) * ( b.GetPosOfPointOnBody( osim_pos ) - bi.com_world );
		}
		else return osim_pos;
	}

	inline Vec3d translated_inertia( const Vec3d& in, Real m, const Vec3d& o ) {
		return Vec3d(
			in.x + m * ( o.y * o.y + o.z * o.z ),
			in.y + m * ( o.x * o.x + o.z * o.z ),
			in.z + m * ( o.x * o.x + o.y * o.y ) );
	}

	ModelConverter::BodyInfo ModelConverter::GetCompoundedMass( const BodyInfo& m1, const BodyInfo& m2 ) const
	{
		BodyInfo m( m1 );
		m.mass = m1.mass + m2.mass;
		m.com_world = m1.mass / m.mass * m1.com_world + m2.mass / m.mass * m2.com_world;
		m.inertia = translated_inertia( m1.inertia, m1.mass, m.com_world - m1.com_world ) +
			translated_inertia( m2.inertia, m2.mass, m.com_world - m2.com_world );
		return m;
	}

	void ModelConverter::MakeBodyInfo( const Body& b )
	{
		if ( IsCompoundChild( b ) )
		{
			auto& crb = GetCompoundRoot( b );
			auto it = global_bodies_.find( crb.GetName() );
			if ( it == global_bodies_.end() )
				it = global_bodies_.insert( { crb.GetName(), BodyInfo( crb ) } ).first;
			auto& gpb = it->second;
			auto new_gpb = GetCompoundedMass( gpb, BodyInfo( b ) );
			log::info( "Compounded ", b.GetName(), " to ", new_gpb.name, "; com_ofs=", new_gpb.com_world - gpb.com_world );
			gpb = new_gpb;
		}
		else if ( !global_bodies_.contains( b.GetName() ) )
			global_bodies_.insert( { b.GetName() , BodyInfo( b ) } );
	}

	bool ModelConverter::IsCompoundChild( const Body& b ) const
	{
		auto* j = b.GetJoint();
		return j && IsRealJoint( *j ) && j->GetDofs().empty()
			&& compound_welded_bodies && b.GetMass() < compound_mass_threshold;
	}

	const Body& ModelConverter::GetCompoundRoot( const Body& b ) const
	{
		if ( !IsCompoundChild( b ) )
			return b;
		else if ( auto* j = b.GetJoint() )
			return GetCompoundRoot( j->GetParentBody() );
		else SCONE_ERROR( "Could not find compound root for body " + b.GetName() );
	}
}

