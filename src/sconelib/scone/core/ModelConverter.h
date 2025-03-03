#pragma once

#include "scone/model/Model.h"
#include <string>
#include "xo/container/flat_set.h"
#include "xo/container/flat_map.h"

namespace scone
{
	class SCONE_API ModelConverter
	{
	public:
		ModelConverter( const PropNode& settings = {} );
		virtual ~ModelConverter() = default;

		PropNode ConvertModel( Model& m );

		double joint_stiffness_ = 1e6;
		double joint_limit_stiffness_ = 500.0;
		bool use_stiffness_from_limit_force_ = true;
		bool use_limits_from_dof_range_ = false;
		double joint_limit_damping_angle_ = 5.0;
		bool use_body_mass_threshold_ = true;
		double body_mass_threshold_ = 0.1;
		bool compound_welded_bodies = false;
		double compound_mass_threshold = 10.0;
		bool convert_ligaments = true;
		bool use_pin_joints_ = false;
		bool keep_body_origin_ = false;

	private:
		using StringSet = xo::flat_set<std::string>;
		void ConvertBody( const Body& b, PropNode& model_pn, StringSet& converted ) const;
		void ConvertJoint( const Joint& b, PropNode& parent_pn ) const;
		void ConvertMuscle( const Muscle& m, PropNode& parent_pn ) const;
		void ConvertLigament( const Ligament& l, PropNode& parent_pn ) const;
		void ConvertContactGeometry( const ContactGeometry& j, PropNode& body_pn ) const;
		void ConvertDisplayGeometry( const Body& b, PropNode& model_pn ) const;
		void ConvertDof( const Dof& d, PropNode& parent_pn ) const;
		void ConvertMaterials( const Model& m, PropNode& parent_pn ) const;
		PropNode& FindBodyPropNode( const Body& m, PropNode& model_pn ) const;

		struct BodyInfo {
			BodyInfo( const Body& b ) :
				name( b.GetName() ),
				mass( b.GetMass() ),
				inertia( b.GetInertiaTensorDiagonal() ),
				com_world( b.GetComPos() ),
				ori_world( b.GetOrientation() )
			{}
			std::string name;
			Real mass;
			Vec3d inertia;
			Vec3d com_world;
			Quat ori_world;
		};

		bool IsCompoundChild( const Body& b ) const;
		const Body& GetCompoundRoot( const Body& b ) const;
		void MakeBodyInfo( const Body& b );

		const BodyInfo& GetGlobalBody( const Body& b ) const;
		const String& GetBodyName( const Body& b ) const { return GetGlobalBody( b ).name; }
		Vec3 GetLocalBodyPos( const Vec3& osim_pos, const Body& b ) const;
		BodyInfo GetCompoundedMass( const BodyInfo& m1, const BodyInfo& m2 ) const;

		xo::flat_map<std::string, BodyInfo> global_bodies_;
	};
}

