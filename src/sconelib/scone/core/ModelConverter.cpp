#include "ModelConverter.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/Joint.h"
#include "scone/model/Muscle.h"

namespace scone
{
	ModelConverter::ModelConverter( const PropNode& pn ) :
		INIT_MEMBER( pn, joint_stiffness_, 1e6 ),
		INIT_MEMBER( pn, joint_limit_stiffness_, 500 ),
		INIT_MEMBER( pn, body_mass_threshold_, 0.2 )
	{}

	PropNode ModelConverter::ConvertModel( const Model& model )
	{
		PropNode pn;
		auto& model_pn = pn.add_child( "model" );

		for ( auto b : model.GetBodies() )
			model_pn.append( ConvertBody( *b ) );

		for ( auto m : model.GetMuscles() )
			model_pn.append( ConvertMuscle( *m ) );

		return pn;
	}

	PropNode ModelConverter::ConvertBody( const Body& b )
	{
		PropNode pn;
		auto& body_pn = pn.add_child( "body" );
		body_pn[ "name" ] = b.GetName();
		body_pn[ "mass" ] = b.GetMass();
		body_pn[ "inertia" ] = b.GetInertiaTensorDiagonal();
		if ( auto* j = b.GetJoint() )
		{
			auto& bp = j->GetParentBody();
			auto& bc = j->GetBody();
			auto tfp = xo::transformd{ bp.GetComPos(), bp.GetOrientation() };
			auto tfc = xo::transformd{ bc.GetComPos(), bc.GetOrientation() };

			auto& joint_pn = body_pn.add_child( "joint" );
			joint_pn[ "name" ] = j->GetName();
			joint_pn[ "parent" ] = j->GetParentBody().GetName();
			joint_pn[ "pos_in_parent" ] = tfp.inv_trans( j->GetPos() );
			joint_pn[ "pos_in_child" ] = tfc.inv_trans( j->GetPos() );
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
}
