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
	{
	}

	PropNode ModelConverter::ConvertModel( const Model& m )
	{
		PropNode pn;
		auto& model_pn = pn.add_child( "model" );
		for ( auto b : m.GetBodies() )
			ConvertBody( *b );

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
			auto& joint_pn = pn.add_child( "joint" );
			joint_pn[ "parent" ] = j->GetParentBody().GetName();
			joint_pn[ "pos_in_parent" ] = j->GetPos();
			joint_pn[ "pos_in_child" ] = j->GetPos();
		}

		return pn;
	}

	PropNode ModelConverter::ConvertJoint( const Joint& j )
	{
		return PropNode();
	}

	PropNode ModelConverter::ConvertMuscle( const Muscle& m )
	{
		PropNode pn;
		auto& mus_pn = pn.add_child( "point_path_muscle" );
		mus_pn[ "name" ] = m.GetName();
		mus_pn[ "max_isometric_force" ] = m.GetMaxIsometricForce();
		mus_pn[ "optimal_fiber_length" ] = m.GetOptimalFiberLength();
		mus_pn[ "tendon_slack_length" ] = m.GetTendonSlackLength();
		mus_pn[ "pennation_angle" ] = 0.0;

		return pn;
	}
}
