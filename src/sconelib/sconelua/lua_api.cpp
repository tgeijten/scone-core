#include "lua_api.h"

#include "sol/sol.hpp"
#include "xo_lua/lua_register.h"

namespace scone
{
	void register_lua_wrappers( sol::state& lua )
	{
		xo::lua_register_vec3<double>( lua, "vec3" );
		xo::lua_register_quat<double>( lua, "quat" );

		lua.new_usertype<LuaFrame>( "LuaFrame", sol::constructors<>(),
			"set_value", &LuaFrame::set_value,
			"set_bool", &LuaFrame::set_bool,
			"time", &LuaFrame::time
			);

		lua.new_usertype<LuaActuator>( "LuaActuator", sol::constructors<>(),
			"name", &LuaActuator::name,
			"add_input", &LuaActuator::add_input,
			"input", &LuaActuator::input,
			"min_input", &LuaActuator::min_input,
			"max_input", &LuaActuator::max_input
			);

		lua.new_usertype<LuaDof>( "LuaDof", sol::constructors<>(),
			"name", &LuaDof::name,
			"position", &LuaDof::position,
			"velocity", &LuaDof::velocity,
			"is_actuated", &LuaDof::is_actuated,
			"add_input", &LuaDof::add_input,
			"input", &LuaDof::input,
			"min_input", &LuaDof::min_input,
			"max_input", &LuaDof::max_input,
			"min_torque", &LuaDof::min_torque,
			"max_torque", &LuaDof::max_torque,
			"muscle_moment", &LuaDof::muscle_moment
			);

		lua.new_usertype<LuaBody>( "LuaBody", sol::constructors<>(),
			"name", &LuaBody::name,
			"mass", &LuaBody::mass,
			"inertia_diagonal", &LuaBody::inertia_diagonal,
			"com_pos", &LuaBody::com_pos,
			"com_vel", &LuaBody::com_vel,
			"com_acc", &LuaBody::com_acc,
			"point_pos", &LuaBody::point_pos,
			"point_vel", &LuaBody::point_vel,
			"ori", &LuaBody::ori,
			"ang_pos", &LuaBody::ang_pos,
			"ang_vel", &LuaBody::ang_vel,
			"ang_acc", &LuaBody::ang_acc,
			"contact_force", &LuaBody::contact_force,
			"contact_moment", &LuaBody::contact_moment,
			"contact_point", &LuaBody::contact_point,
			"add_external_force", &LuaBody::add_external_force,
			"add_external_moment", &LuaBody::add_external_moment,
			"set_com_pos", &LuaBody::set_com_pos,
			"set_ori", &LuaBody::set_ori,
			"set_lin_vel", &LuaBody::set_lin_vel,
			"set_ang_vel", &LuaBody::set_ang_vel
			);

		lua.new_usertype<LuaJoint>( "LuaJoint", sol::constructors<>(),
			"name", &LuaJoint::name,
			"pos", &LuaJoint::pos,
			"has_motor", &LuaJoint::has_motor,
			"set_motor_target_ori", &LuaJoint::set_motor_target_ori,
			"set_motor_target_vel", &LuaJoint::set_motor_target_vel,
			"add_motor_torque", &LuaJoint::add_motor_torque,
			"set_motor_stiffness", &LuaJoint::set_motor_stiffness,
			"set_motor_damping", &LuaJoint::set_motor_damping
			);

		lua.new_usertype<LuaMuscle>( "LuaMuscle", sol::constructors<>(),
			"name", &LuaMuscle::name,
			"add_input", &LuaMuscle::add_input,
			"input", &LuaMuscle::input,
			"excitation", &LuaMuscle::excitation,
			"activation", &LuaMuscle::activation,
			"fiber_length", &LuaMuscle::fiber_length,
			"normalized_fiber_length", &LuaMuscle::normalized_fiber_length,
			"optimal_fiber_length", &LuaMuscle::optimal_fiber_length,
			"tendon_length", &LuaMuscle::tendon_length,
			"normalized_tendon_length", &LuaMuscle::normalized_tendon_length,
			"tendon_slack_length", &LuaMuscle::tendon_slack_length,
			"muscle_tendon_length", &LuaMuscle::muscle_tendon_length,
			"muscle_tendon_velocity", &LuaMuscle::muscle_tendon_velocity,
			"force", &LuaMuscle::force,
			"normalized_force", &LuaMuscle::normalized_force,
			"max_isometric_force", &LuaMuscle::max_isometric_force,
			"fiber_velocity", &LuaMuscle::fiber_velocity,
			"normalized_fiber_velocity", &LuaMuscle::normalized_fiber_velocity
			);

		lua.new_usertype<LuaModel>( "LuaModel", sol::constructors<>(),
			"time", &LuaModel::time,
			"delta_time", &LuaModel::delta_time,
			"max_duration", &LuaModel::max_duration,
			"com_pos", &LuaModel::com_pos,
			"com_vel", &LuaModel::com_vel,
			"actuator", &LuaModel::actuator,
			"find_actuator", &LuaModel::find_actuator,
			"actuator_count", &LuaModel::actuator_count,
			"dof", &LuaModel::dof,
			"find_dof", &LuaModel::find_dof,
			"dof_count", &LuaModel::dof_count,
			"muscle", &LuaModel::muscle,
			"find_muscle", &LuaModel::find_muscle,
			"muscle_count", &LuaModel::muscle_count,
			"body", &LuaModel::body,
			"find_body", &LuaModel::find_body,
			"body_count", &LuaModel::body_count,
			"ground_body", &LuaModel::ground_body,
			"joint", &LuaModel::joint,
			"find_joint", &LuaModel::find_joint,
			"joint_count", &LuaModel::joint_count
			);

		lua.new_usertype<LuaParams>( "LuaParams", sol::constructors<>(),
			"create_from_mean_std", &LuaParams::create_from_mean_std,
			"create_from_string", &LuaParams::create_from_string
			);

		lua.new_usertype<LuaScone>( "scone", sol::constructors<>(),
			"trace", &LuaScone::trace,
			"debug", &LuaScone::debug,
			"info", &LuaScone::info,
			"warning", &LuaScone::warning,
			"error", &LuaScone::error,
			"quat_from_euler_deg", &LuaScone::quat_from_euler_deg
			);
	}
}
