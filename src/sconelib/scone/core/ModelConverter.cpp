#include "ModelConverter.h"
#include "xo/container/prop_node_tools.h"

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
		
		return pn;
	}

	PropNode ModelConverter::ConvertJoint( const Joint& j )
	{
		return PropNode();
	}

	PropNode ModelConverter::ConvertMuscle( const Muscle& j )
	{
		return PropNode();
	}
}
