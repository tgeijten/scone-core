#include "lua_api.h"

#include "xo_lua/lua_register.h"

namespace scone
{
	void register_lua_wrappers( sol::state& lua )
	{
		xo::lua_register_vec3<double>( lua, "vec3" );
		xo::lua_register_quat<double>( lua, "quat" );

		lua.new_usertype<LuaFrame>( "LuaFrame", sol::constructors<>(),
			"set_value", &LuaFrame::set_value,
			"set_vec3", &LuaFrame::set_vec3,
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

		lua.new_usertype<LuaSensor>( "LuaSensor", sol::constructors<>(),
			"value", &LuaSensor::value
			);

		lua.new_usertype<LuaDelayedSensor>( "LuaDelayedSensor", sol::constructors<>(),
			"value", &LuaDelayedSensor::value,
			"delay_buffer_size", &LuaDelayedSensor::delay_buffer_size
			);

		lua.new_usertype<LuaDelayedActuator>( "LuaDelayedActuator", sol::constructors<>(),
			"add_input", &LuaDelayedActuator::add_input,
			"delay_buffer_size", &LuaDelayedActuator::delay_buffer_size
			);

		lua.new_usertype<LuaDof>( "LuaDof", sol::constructors<>(),
			"name", &LuaDof::name,
			"position", &LuaDof::position,
			"velocity", &LuaDof::velocity,
			"is_rotational", &LuaDof::is_rotational,
			"rotation_axis", &LuaDof::rotation_axis,
			"is_actuated", &LuaDof::is_actuated,
			"add_input", &LuaDof::add_input,
			"input", &LuaDof::input,
			"min_input", &LuaDof::min_input,
			"max_input", &LuaDof::max_input,
			"min_torque", &LuaDof::min_torque,
			"max_torque", &LuaDof::max_torque,
			"actuator_torque", &LuaDof::actuator_torque,
			"muscle_moment", &LuaDof::muscle_moment,
			"set_position", &LuaDof::set_position,
			"set_velocity", &LuaDof::set_velocity,
			"create_delayed_position_sensor", &LuaDof::create_delayed_position_sensor,
			"create_delayed_velocity_sensor", &LuaDof::create_delayed_velocity_sensor
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
			"reaction_force", &LuaJoint::reaction_force,
			"limit_torque", &LuaJoint::limit_torque,
			"limit_power", &LuaJoint::limit_power,
			"load", &LuaJoint::load,
			"has_motor", &LuaJoint::has_motor,
			"set_motor_target_ori", &LuaJoint::set_motor_target_ori,
			"set_motor_target_vel", &LuaJoint::set_motor_target_vel,
			"add_motor_torque", &LuaJoint::add_motor_torque,
			"set_motor_stiffness", &LuaJoint::set_motor_stiffness,
			"set_motor_damping", &LuaJoint::set_motor_damping
			);

		lua.new_usertype<LuaSpring>( "LuaSpring", sol::constructors<>(),
			"parent_body", &LuaSpring::parent_body,
			"child_body", &LuaSpring::child_body,
			"pos_in_parent", &LuaSpring::pos_in_parent,
			"pos_in_child", &LuaSpring::pos_in_child,
			"rest_length", &LuaSpring::rest_length,
			"stiffness", &LuaSpring::stiffness,
			"damping", &LuaSpring::damping,
			"parent_pos", &LuaSpring::parent_pos,
			"child_pos", &LuaSpring::child_pos,
			"is_active", &LuaSpring::is_active,
			"set_parent", &LuaSpring::set_parent,
			"set_child", &LuaSpring::set_child,
			"set_rest_length", &LuaSpring::set_rest_length,
			"set_stiffness", &LuaSpring::set_stiffness,
			"set_damping", &LuaSpring::set_damping
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
			"pennation_angle_at_optimal", &LuaMuscle::pennation_angle_at_optimal,
			"fiber_velocity", &LuaMuscle::fiber_velocity,
			"normalized_fiber_velocity", &LuaMuscle::normalized_fiber_velocity,
			"max_contraction_velocity", &LuaMuscle::max_contraction_velocity,
			"tendon_length", &LuaMuscle::tendon_length,
			"normalized_tendon_length", &LuaMuscle::normalized_tendon_length,
			"tendon_slack_length", &LuaMuscle::tendon_slack_length,
			"muscle_tendon_length", &LuaMuscle::muscle_tendon_length,
			"muscle_tendon_velocity", &LuaMuscle::muscle_tendon_velocity,
			"force", &LuaMuscle::force,
			"normalized_force", &LuaMuscle::normalized_force,
			"active_fiber_force", &LuaMuscle::active_fiber_force,
			"active_force_length_multiplier", &LuaMuscle::active_force_length_multiplier,
			"force_velocity_multiplier", &LuaMuscle::force_velocity_multiplier,
			"passive_fiber_force", &LuaMuscle::passive_fiber_force,
			"cos_pennation_angle", &LuaMuscle::cos_pennation_angle,
			"max_isometric_force", &LuaMuscle::max_isometric_force,
			"mass", &LuaMuscle::mass,
			"moment_arm", &LuaMuscle::moment_arm,
			"moment_arm_3d", &LuaMuscle::moment_arm_3d,
			"set_max_activation", &LuaMuscle::set_max_activation,
			"set_min_activation", &LuaMuscle::set_min_activation,
			"create_delayed_force_sensor", &LuaMuscle::create_delayed_force_sensor,
			"create_delayed_length_sensor", &LuaMuscle::create_delayed_length_sensor,
			"create_delayed_velocity_sensor", &LuaMuscle::create_delayed_velocity_sensor,
			"create_delayed_activation_sensor", &LuaMuscle::create_delayed_activation_sensor,
			"create_delayed_actuator", &LuaMuscle::create_delayed_actuator
			);

		lua.new_usertype<LuaModel>( "LuaModel", sol::constructors<>(),
			"time", &LuaModel::time,
			"delta_time", &LuaModel::delta_time,
			"max_duration", &LuaModel::max_duration,
			"com_pos", &LuaModel::com_pos,
			"com_vel", &LuaModel::com_vel,
			"mass", &LuaModel::mass,
			"gravity", &LuaModel::gravity,
			"set_gravity", &LuaModel::set_gravity,
			"actuator", &LuaModel::actuator,
			"find_actuator", &LuaModel::find_actuator,
			"actuator_count", &LuaModel::actuator_count,
			"dof", &LuaModel::dof,
			"find_dof", &LuaModel::find_dof,
			"dof_count", &LuaModel::dof_count,
			"init_state_from_dofs", &LuaModel::init_state_from_dofs,
			"muscle", &LuaModel::muscle,
			"find_muscle", &LuaModel::find_muscle,
			"muscle_count", &LuaModel::muscle_count,
			"body", &LuaModel::body,
			"find_body", &LuaModel::find_body,
			"find_body_index", &LuaModel::find_body_index,
			"body_count", &LuaModel::body_count,
			"ground_body", &LuaModel::ground_body,
			"joint", &LuaModel::joint,
			"find_joint", &LuaModel::find_joint,
			"joint_count", &LuaModel::joint_count,
			"get_interaction_spring", &LuaModel::get_interaction_spring,
			"set_custom_value", &LuaModel::set_custom_value,
			"get_custom_value", &LuaModel::get_custom_value,
			"has_custom_value", &LuaModel::has_custom_value,
			"find_two_way_neural_delay", &LuaModel::find_two_way_neural_delay,
			"find_one_way_neural_delay", &LuaModel::find_one_way_neural_delay,
			"create_muscle_force_sensor", &LuaModel::create_muscle_force_sensor,
			"create_muscle_length_sensor", &LuaModel::create_muscle_length_sensor,
			"create_muscle_velocity_sensor", &LuaModel::create_muscle_velocity_sensor,
			"create_muscle_activation_sensor", &LuaModel::create_muscle_activation_sensor,
			"create_delayed_muscle_force_sensor", &LuaModel::create_delayed_muscle_force_sensor,
			"create_delayed_muscle_length_sensor", &LuaModel::create_delayed_muscle_length_sensor,
			"create_delayed_muscle_velocity_sensor", &LuaModel::create_delayed_muscle_velocity_sensor,
			"create_delayed_muscle_activation_sensor", &LuaModel::create_delayed_muscle_activation_sensor,
			"create_delayed_muscle_actuator", &LuaModel::create_delayed_muscle_actuator
			);

		lua.new_usertype<LuaController>( "LuaController", sol::constructors<>(),
			"name", &LuaController::name,
			"child_count", &LuaController::child_count,
			"child", &LuaController::child,
			"enable", &LuaController::enable,
			"enabled", &LuaController::enabled,
			"set_child_enabled", &LuaController::set_child_enabled,
			"is_child_enabled", &LuaController::is_child_enabled,
			"set_control_parameter", &LuaController::set_control_parameter,
			"get_control_parameter", &LuaController::get_control_parameter,
			"get_control_parameter_names", &LuaController::get_control_parameter_names,
			"create_control_parameter", &LuaController::create_control_parameter
			);

		lua.new_usertype<LuaParams>( "LuaParams", sol::constructors<>(),
			"create_from_mean_std", &LuaParams::create_from_mean_std,
			"create_from_string", &LuaParams::create_from_string,
			"get", &LuaParams::get
			);

		lua.new_usertype<LuaScone>( "scone", sol::constructors<>(),
			"trace", &LuaScone::trace,
			"debug", &LuaScone::debug,
			"info", &LuaScone::info,
			"warning", &LuaScone::warning,
			"error", &LuaScone::error,
			"quat_from_euler_deg", &LuaScone::quat_from_euler_deg,
			"quat_from_euler_rad", &LuaScone::quat_from_euler_rad
			);
	}
}
