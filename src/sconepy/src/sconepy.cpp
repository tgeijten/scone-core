#include "sconepy.h"
#include "sconepy_tools.h"
#include "sconepy_sensors.h"

#include "scone/core/version.h"
#include "xo/system/log_sink.h"
#include "scone/sconelib_config.h"
#include "scone/model/Model.h"
#include "scone/model/Muscle.h"
#include "scone/model/Actuator.h"
#include "scone/model/Dof.h"
#include "scone/model/Joint.h"

PYBIND11_MODULE( sconepy, m ) {
	static xo::log::console_sink console_sink( xo::log::level::trace );
	static bool g_f32 = true;

	scone::Initialize();

	m.attr( "__version__" ) = "0.1.0";

	m.def( "version", []() { return xo::to_str( scone::GetSconeVersion() ); } );
	m.def( "scone_dir", []() { return scone::GetFolder( scone::SconeFolder::Scenarios ).str(); } );
	m.def( "set_log_level", []( int l ) { console_sink.set_log_level( xo::log::level( l ) ); } );
	m.def( "evaluate_par_file", &scone::evaluate_par_file );
	m.def( "load_model", &scone::load_model );
	m.def( "is_supported", &scone::is_supported );
	m.def( "replace_string_tags", &scone::ReplaceStringTags );

	m.def( "set_array_dtype_float32", []() { g_f32 = true; } );
	m.def( "set_array_dtype_float64", []() { g_f32 = false; } );
	m.def( "is_array_dtype_float32", []() -> bool { return g_f32; } );
	m.def( "is_array_dtype_float64", []() -> bool { return !g_f32; } );

	py::class_<scone::Vec3>( m, "Vec3" )
		.def_readwrite( "x", &scone::Vec3::x )
		.def_readwrite( "y", &scone::Vec3::y )
		.def_readwrite( "z", &scone::Vec3::z )
		.def( "array", []( scone::Vec3& v ) { return to_array( std::vector{ v.x, v.y, v.z }, g_f32 ); } )
		.def( "__repr__", []( const scone::Vec3& v ) { return xo::stringf( "<sconepy.Vec3( %f, %f, %f)>", v.x, v.y, v.z ); } )
		.def( "__str__", []( const scone::Vec3& v ) { return xo::stringf( "[ %f %f %f ]", v.x, v.y, v.z ); } )
		;

	py::class_<scone::Quat>( m, "Quat" )
		.def_readwrite( "w", &scone::Quat::w )
		.def_readwrite( "x", &scone::Quat::x )
		.def_readwrite( "y", &scone::Quat::y )
		.def_readwrite( "z", &scone::Quat::z )
		.def( "array", []( scone::Quat& q ) { return to_array( std::vector{ q.w, q.x, q.y, q.z }, g_f32 ); } )
		.def( "to_euler_xyz", []( scone::Quat& q ) { return euler_xyz_from_quat( q ); } )
		.def( "to_euler_xzy", []( scone::Quat& q ) { return euler_xzy_from_quat( q ); } )
		.def( "to_euler_yxz", []( scone::Quat& q ) { return euler_yxz_from_quat( q ); } )
		.def( "to_euler_yzx", []( scone::Quat& q ) { return euler_yzx_from_quat( q ); } )
		.def( "to_euler_zxy", []( scone::Quat& q ) { return euler_zxy_from_quat( q ); } )
		.def( "to_euler_zyx", []( scone::Quat& q ) { return euler_zyx_from_quat( q ); } )
		.def( "to_rotation_vector", []( scone::Quat& q ) { return rotation_vector_from_quat( q ); } )
		;

	py::class_<scone::Body>( m, "Body" )
		.def( "name", &scone::Body::GetName )
		.def( "mass", &scone::Body::GetMass )
		.def( "inertia_diag", &scone::Body::GetInertiaTensorDiagonal )
		.def( "com_pos", &scone::Body::GetComPos )
		.def( "com_vel", &scone::Body::GetComVel )
		.def( "orientation", &scone::Body::GetOrientation )
		.def( "ang_vel", &scone::Body::GetAngVel )
		.def( "set_pos", &scone::Body::SetPos )
		.def( "set_lin_vel", &scone::Body::SetLinVel )
		.def( "set_orientation", &scone::Body::SetOrientation )
		.def( "set_ang_vel", &scone::Body::SetAngVel )
		.def( "contact_force", &scone::Body::GetContactForce )
		.def( "contact_moment", &scone::Body::GetContactMoment )
		.def( "contact_point", &scone::Body::GetContactPoint )
		.def( "set_external_force", &scone::Body::SetExternalForce )
		.def( "set_external_force_at", &scone::Body::SetExternalForceAtPoint )
		.def( "set_external_moment", &scone::Body::SetExternalMoment )
		.def( "add_external_force", &scone::Body::AddExternalForce )
		.def( "add_external_moment", &scone::Body::AddExternalMoment )
		.def( "clear_external_force_moment", &scone::Body::ClearExternalForceAndMoment )
		.def( "external_force", &scone::Body::GetExternalForce )
		.def( "external_moment", &scone::Body::GetExternalMoment )
		.def( "external_force_at", &scone::Body::GetExternalForcePoint )
		;

	py::class_<scone::Joint>( m, "Joint" )
		.def( "name", &scone::Joint::GetName )
		.def( "reaction_force", &scone::Joint::GetReactionForce )
		.def( "limit_torque", &scone::Joint::GetLimitTorque )
		.def( "limit_power", &scone::Joint::GetLimitPower )
		.def( "load", &scone::Joint::GetLoad )
		.def( "body", &scone::Joint::GetBody, py::return_value_policy::reference )
		.def( "parent_body", &scone::Joint::GetParentBody, py::return_value_policy::reference )
		.def( "has_motor", &scone::Joint::HasMotor )
		.def( "motor_max_torque", &scone::Joint::GetMotorMaxTorque )
		.def( "set_motor_target_orientation", &scone::Joint::SetMotorTargetOri )
		.def( "set_motor_target_velocity", &scone::Joint::SetMotorTargetVel )
		.def( "add_motor_torque", &scone::Joint::AddMotorTorque )
		.def( "set_motor_stiffness", &scone::Joint::SetMotorStiffness )
		.def( "set_motor_damping", &scone::Joint::SetMotorDamping )
		;

	py::class_<scone::Dof>( m, "Dof" )
		.def( "name", &scone::Dof::GetName )
		.def( "pos", &scone::Dof::GetPos )
		.def( "vel", &scone::Dof::GetVel )
		.def( "set_pos", &scone::Dof::SetPos )
		.def( "set_vel", &scone::Dof::SetVel )
		.def( "is_rotational", &scone::Dof::IsRotational )
		.def( "rotation_axis", &scone::Dof::GetRotationAxis )
		.def( "is_actuated", &scone::Dof::IsActuated )
		;

	py::class_<scone::Actuator>( m, "Actuator" )
		.def( "name", &scone::Actuator::GetName )
		.def( "input", &scone::Actuator::GetInput )
		.def( "min_input", &scone::Actuator::GetMinInput )
		.def( "max_input", &scone::Actuator::GetMaxInput )
		.def( "add_input", &scone::Actuator::AddInput )
		.def( "clear_input", &scone::Actuator::ClearInput )
		;

	py::class_<scone::Muscle>( m, "Muscle" )
		.def( "name", &scone::Muscle::GetName )
		.def( "max_isometric_force", &scone::Muscle::GetMaxIsometricForce )
		.def( "optimal_fiber_length", &scone::Muscle::GetOptimalFiberLength )
		.def( "tendon_slack_length", &scone::Muscle::GetTendonSlackLength )
		.def( "origin_body", &scone::Muscle::GetOriginBody, py::return_value_policy::reference )
		.def( "insertion_body", &scone::Muscle::GetInsertionBody, py::return_value_policy::reference )
		.def( "tendon_slack_length", &scone::Muscle::GetTendonSlackLength )
		.def( "excitation", &scone::Muscle::GetExcitation )
		.def( "activation", &scone::Muscle::GetActivation )
		.def( "set_excitation", &scone::Muscle::SetExcitation )
		.def( "init_activation", &scone::Muscle::InitializeActivation )
		.def( "force", &scone::Muscle::GetForce )
		.def( "force_norm", &scone::Muscle::GetNormalizedForce )
		.def( "fiber_length", &scone::Muscle::GetFiberLength )
		.def( "fiber_length_norm", &scone::Muscle::GetNormalizedFiberLength )
		.def( "fiber_velocity", &scone::Muscle::GetFiberVelocity )
		.def( "fiber_velocity_norm", &scone::Muscle::GetNormalizedFiberVelocity )
		;

	py::class_<scone::Leg>( m, "Leg" )
		.def( "name", &scone::Leg::GetName )
		.def( "contact_force", &scone::Leg::GetContactForce )
		.def( "contact_load", &scone::Leg::GetLoad )
		.def( "upper_body", &scone::Leg::GetUpperBody, py::return_value_policy::reference )
		.def( "foot_body", &scone::Leg::GetFootBody, py::return_value_policy::reference )
		.def( "base_body", &scone::Leg::GetBaseBody, py::return_value_policy::reference )
		.def( "relative_foot_position", &scone::Leg::GetRelFootPos )
		.def( "length", &scone::Leg::GetLength )
		;

	py::class_<scone::Measure>( m, "Measure" )
		.def( "name", &scone::Measure::GetName )
		.def( "final_result", &scone::Measure::GetResult )
		.def( "final_weighted_result", &scone::Measure::GetWeightedResult )
		.def( "current_result", &scone::Measure::GetCurrentResult )
		.def( "current_weighted_result", &scone::Measure::GetCurrentWeightedResult )
		;

	py::class_<scone::Model>( m, "Model" )
		.def( "name", &scone::Model::GetName )
		.def( "com_pos", &scone::Model::GetComPos )
		.def( "com_vel", &scone::Model::GetComVel )
		.def( "mass", &scone::Model::GetMass )
		.def( "gravity", &scone::Model::GetGravity )
		.def( "contact_force", &scone::Model::GetTotalContactForce )
		.def( "contact_load", []( scone::Model& m ) { return m.GetTotalContactForce() / m.GetBW(); } )
		.def( "contact_power", &scone::Model::GetTotalContactPower )
		.def( "state", &scone::get_state )
		.def( "set_state", &scone::set_state )
		.def( "bodies", []( scone::Model& m ) { return &m.GetBodies(); }, py::return_value_policy::reference )
		.def( "joints", []( scone::Model& m ) { return &m.GetJoints(); }, py::return_value_policy::reference )
		.def( "dofs", []( scone::Model& m ) { return &m.GetDofs(); }, py::return_value_policy::reference )
		.def( "actuators", []( scone::Model& m ) { return &m.GetActuators(); }, py::return_value_policy::reference )
		.def( "muscles", []( scone::Model& m ) { return &m.GetMuscles(); }, py::return_value_policy::reference )
		.def( "legs", []( scone::Model& m ) { return &m.GetLegs(); }, py::return_value_policy::reference )
		.def( "measure", []( scone::Model& m ) { return m.GetMeasure(); }, py::return_value_policy::reference )
		.def( "set_dof_positions", &scone::set_dof_positions )
		.def( "set_dof_velocities", &scone::set_dof_velocities )
		.def( "init_state_from_dofs", []( scone::Model& m ) { m.UpdateStateFromDofs(); } )
		.def( "adjust_state_for_load", &scone::Model::AdjustStateForLoad )
		.def( "set_actuator_inputs", &scone::set_actuator_inputs )
		.def( "actuator_input_array", []( scone::Model& m ) { return scone::get_actuator_inputs( m, g_f32 ); } )
		.def( "dof_position_array", []( scone::Model& m ) { return scone::get_dof_positions( m, g_f32 ); } )
		.def( "dof_velocity_array", []( scone::Model& m ) { return scone::get_dof_velocities( m, g_f32 ); } )
		.def( "muscle_fiber_length_array", []( scone::Model& m ) { return scone::get_muscle_lengths( m, g_f32 ); } )
		.def( "muscle_fiber_velocity_array", []( scone::Model& m ) { return scone::get_muscle_velocities( m, g_f32 ); } )
		.def( "muscle_force_array", []( scone::Model& m ) { return scone::get_muscle_forces( m, g_f32 ); } )
		.def( "muscle_activation_array", []( scone::Model& m ) { return scone::get_muscle_activations( m, g_f32 ); } )
		.def( "muscle_excitation_array", []( scone::Model& m ) { return scone::get_muscle_excitations( m, g_f32 ); } )
		.def( "delayed_muscle_fiber_length_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::Length, g_f32 ); } )
		.def( "delayed_muscle_fiber_velocity_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::Velocity, g_f32 ); } )
		.def( "delayed_muscle_force_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::Force, g_f32 ); } )
		.def( "delayed_dof_position_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::DofPos, g_f32 ); } )
		.def( "delayed_dof_velocity_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::DofVel, g_f32 ); } )
		.def( "delayed_vestibular_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::Vestibular, g_f32 ); } )
		.def( "set_delayed_actuator_inputs", &scone::set_delayed_actuator_inputs )
		.def( "init_muscle_activations", &scone::init_muscle_activations )
		.def( "advance_simulation_to", &scone::Model::AdvanceSimulationTo )
		.def( "reset", &scone::Model::Reset )
		.def( "time", &scone::Model::GetTime )
		.def( "set_simulation_end_time", &scone::Model::SetSimulationEndTime )
		.def( "has_simulation_ended", &scone::Model::HasSimulationEnded )
		.def( "current_measure", &scone::Model::GetCurrentMeasureResult )
		.def( "final_measure", &scone::Model::GetMeasureResult )
		.def( "log_measure_report", []( scone::Model& m ) { return scone::log_measure_report( m ); } )
		.def( "integration_step", &scone::Model::GetIntegrationStep )
		.def( "control_step_size", []( scone::Model& m ) { return m.fixed_control_step_size; } )
		.def( "set_store_data", &scone::Model::SetStoreData )
		.def( "write_results", &scone::write_results )
		;
}
