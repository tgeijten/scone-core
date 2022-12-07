#pragma once

#include "scone/model/Model.h"

namespace scone
{
	class SCONE_API ModelConverter
	{
	public:
		ModelConverter( const PropNode& settings = {} );
		virtual ~ModelConverter() = default;

		PropNode ConvertModel( const Model& m );

		double joint_stiffness_ = 1e6;
		double joint_limit_stiffness_ = 500;
		double body_mass_threshold_ = 0.2;
		
	private:
		PropNode ConvertBody( const Body& b );
		PropNode ConvertMuscle( const Muscle& j );
	};
}
