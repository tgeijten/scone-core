#include "sconepy.h"
#include "sconepy_tools.h"
#include "sconepy_sensors.h"
#include <pybind11/operators.h>

#include "scone/core/version.h"
#include "xo/system/log_sink.h"
#include "scone/sconelib_config.h"
#include "scone/model/Model.h"
#include "scone/model/Muscle.h"
#include "scone/model/Actuator.h"
#include "scone/model/Dof.h"
#include "scone/model/Joint.h"
#include "scone/core/Angle.h"
#include "scone/core/Quat.h"
#include "scone/optimization/Optimizer.h"
#include "sconepy_scenario.h"

PYBIND11_MODULE( sconepy, m ) {
	static xo::log::console_sink console_sink( xo::log::level::trace );
	static bool g_f32 = true;

	scone::Initialize();

	m.attr( "__version__" ) = "1.0.0";

	py::class_<scone::Vec3>( m, "Vec3" )
		.def( py::init<>() )
		.def( py::init<scone::Real, scone::Real, scone::Real>() )
		.def_readwrite( "x", &scone::Vec3::x, "X component" )
		.def_readwrite( "y", &scone::Vec3::y, "Y component" )
		.def_readwrite( "z", &scone::Vec3::z, "Z component" )
		.def( py::self + scone::Vec3(), "Addition" )
		.def( py::self - scone::Vec3(), "Subtraction" )
		.def( py::self * scone::Real(), "Scalar multiplication" )
		.def( scone::Real() * py::self, "Scalar multiplication" )
		.def( py::self / scone::Real(), "Scalar division" )
		.def( -py::self, "Negation" )
		.def( "length", []( const scone::Vec3& v ) { return xo::length( v ); }, "Get the length of this Vec3" )
		.def( "normalize", []( scone::Vec3& v ) { return xo::normalize( v ); }, "Normalize this Vec3" )
		.def( "normalized", []( const scone::Vec3& v ) { return xo::normalized( v ); }, "Get a normalized copy of this Vec3" )
		.def( "dot", []( const scone::Vec3& v1, const scone::Vec3& v2 ) { return xo::dot_product( v1, v2 ); }, "Vec3 dot product" )
		.def( "array", []( scone::Vec3& v ) { return to_array( std::vector{ v.x, v.y, v.z }, g_f32 ); }, "Convert to NumPy array" )
		.def( "__repr__", []( const scone::Vec3& v ) { return xo::stringf( "<sconepy.Vec3(%f, %f, %f)>", v.x, v.y, v.z ); } )
		.def( "__str__", []( const scone::Vec3& v ) { return xo::stringf( "[ %f %f %f ]", v.x, v.y, v.z ); } )
		;

	py::class_<scone::Quat>( m, "Quat" )
		.def( py::init<>() )
		.def( py::init<scone::Real, scone::Real, scone::Real, scone::Real>() )
		.def( py::init( []( scone::Real x, scone::Real y, scone::Real z ) { return xo::quat_from_euler_xyz( scone::Vec3Deg( x, y, z ) ); } ) )
		.def_readwrite( "w", &scone::Quat::w, "W component" )
		.def_readwrite( "x", &scone::Quat::x, "X component" )
		.def_readwrite( "y", &scone::Quat::y, "Y component" )
		.def_readwrite( "z", &scone::Quat::z, "Z component" )
		.def( "__mul__", []( const scone::Quat& q, const scone::Vec3& v ) { return q * v; }, py::is_operator(), "Quaternion Vector multiplication" )
		.def( py::self * scone::Quat(), "Quaternion multiplication" )
		.def( -py::self, "Quaternion conjugate" )
		.def( "normalize", []( scone::Quat& q ) { return xo::normalize( q ); }, "Normalize this Quaternion" )
		.def( "array", []( scone::Quat& q ) { return to_array( std::vector{ q.w, q.x, q.y, q.z }, g_f32 ); }, "Convert to NumPy array" )
		.def( "to_euler_xyz", []( scone::Quat& q ) { return scone::Vec3( scone::Vec3Deg( euler_xyz_from_quat( q ) ) ); }, "Get the XYZ Euler angles of this Quaternion" )
		.def( "to_euler_xzy", []( scone::Quat& q ) { return scone::Vec3( scone::Vec3Deg( euler_xzy_from_quat( q ) ) ); }, "Get the XZY Euler angles of this Quaternion" )
		.def( "to_euler_yxz", []( scone::Quat& q ) { return scone::Vec3( scone::Vec3Deg( euler_yxz_from_quat( q ) ) ); }, "Get the YXZ Euler angles of this Quaternion" )
		.def( "to_euler_yzx", []( scone::Quat& q ) { return scone::Vec3( scone::Vec3Deg( euler_yzx_from_quat( q ) ) ); }, "Get the YZX Euler angles of this Quaternion" )
		.def( "to_euler_zxy", []( scone::Quat& q ) { return scone::Vec3( scone::Vec3Deg( euler_zxy_from_quat( q ) ) ); }, "Get the ZXY Euler angles of this Quaternion" )
		.def( "to_euler_zyx", []( scone::Quat& q ) { return scone::Vec3( scone::Vec3Deg( euler_zyx_from_quat( q ) ) ); }, "Get the ZYX Euler angles of this Quaternion" )
		.def( "to_rotation_vector", []( scone::Quat& q ) { return rotation_vector_from_quat( q ); }, "Convert to rotation vector" )
		.def( "__repr__", []( const scone::Quat& q ) { return xo::stringf( "<sconepy.Quat(%f, %f, %f, %f)>", q.w, q.x, q.y, q.z ); } )
		.def( "__str__", []( const scone::Quat& q ) { return xo::stringf( "[ %f %f %f %f ]", q.w, q.x, q.y, q.z ); } )
		;

	py::class_<scone::Body>( m, "Body" )
		.def( "name", &scone::Body::GetName, "Get the name of this Body" )
		.def( "mass", &scone::Body::GetMass, "Get the mass of this Body" )
		.def( "inertia_diag", &scone::Body::GetInertiaTensorDiagonal, "Get the inertia diagonal of this Body" )
		.def( "com_pos", &scone::Body::GetComPos, "Get the center-of-mass position of this Body" )
		.def( "com_vel", &scone::Body::GetComVel, "Get the center-of-mass velocity of this Body" )
		.def( "com_acc", &scone::Body::GetComAcc, "Get the center-of-mass acceleration of this Body" )
		.def( "orientation", &scone::Body::GetOrientation, "Get the orientation of this Body" )
		.def( "ang_vel", &scone::Body::GetAngVel, "Get the angular velocity of this Body" )
		.def( "ang_acc", &scone::Body::GetAngAcc, "Get the angular acceleration of this Body" )
		.def( "point_pos", &scone::Body::GetPosOfPointOnBody, "Get the world position of a local point on this Body" )
		.def( "point_vel", &scone::Body::GetLinVelOfPointOnBody, "Get the world linear velocity of a local point on this Body" )
		.def( "point_acc", &scone::Body::GetLinAccOfPointOnBody, "Get the world linear acceleration of a local point on this Body" )
		.def( "contact_force", &scone::Body::GetContactForce, "Get the sum of all contact forces acting on this Body" )
		.def( "contact_moment", &scone::Body::GetContactMoment, "Get the sum of all contact moments acting on this Body" )
		.def( "contact_point", &scone::Body::GetContactPoint, "Get the average position of contacts acting on this Body" )
		.def( "external_force", &scone::Body::GetExternalForce, "Get the external perturbation force acting on this Body" )
		.def( "external_moment", &scone::Body::GetExternalMoment, "Get the external perturbation moment acting on this Body" )
		.def( "external_force_at", &scone::Body::GetExternalForcePoint, "Get the position at which the average external force is applied" )
		.def( "set_pos", &scone::Body::SetPos, "Set the position of this Body" )
		.def( "set_lin_vel", &scone::Body::SetLinVel, "Set the linear velocity of this Body" )
		.def( "set_orientation", &scone::Body::SetOrientation, "Set the orientation of this Body" )
		.def( "set_ang_vel", &scone::Body::SetAngVel, "Set the angular velocity of this Body" )
		.def( "set_external_force", &scone::Body::SetExternalForce, "Set the external force at the center-of-mass of this Body" )
		.def( "set_external_force_at", &scone::Body::SetExternalForceAtPoint, "Set the external force at local point on this Body" )
		.def( "set_external_moment", &scone::Body::SetExternalMoment, "Set the external moment of this Body" )
		.def( "add_external_force", &scone::Body::AddExternalForce, "Add an external force to this Body" )
		.def( "add_external_moment", &scone::Body::AddExternalMoment, "Add an external moment to this Body" )
		.def( "clear_external_force_moment", &scone::Body::ClearExternalForceAndMoment, "Clear the external forces and moments applied to this Body" )
		;

	py::class_<scone::Joint>( m, "Joint" )
		.def( "name", &scone::Joint::GetName, "Get the name of this Joint" )
		.def( "pos", &scone::Joint::GetPos, "Get the position of this Joint" )
		.def( "pos_in_parent", &scone::Joint::GetPosInParent, "Get the position of this Joint in the parent Body coordinate frame" )
		.def( "pos_in_child", &scone::Joint::GetPos, "Get the position of this Joint in the child Body coordinate frame" )
		.def( "reaction_force", &scone::Joint::GetReactionForce, "Get the Joint reaction force [N^3]" )
		.def( "limit_torque", &scone::Joint::GetLimitTorque, "Get the Joint limit torque [Nm^3]" )
		.def( "limit_power", &scone::Joint::GetLimitPower, "Get the Joint limit power" )
		.def( "load", &scone::Joint::GetLoad, "Get the scalar joint load [BW]" )
		.def( "body", &scone::Joint::GetBody, py::return_value_policy::reference, "Get the child Body of this Joint" )
		.def( "parent_body", &scone::Joint::GetParentBody, py::return_value_policy::reference, "Get the parent Body of this Joint" )
		.def( "has_motor", &scone::Joint::HasMotor, "See if this Joint has a Joint Motor" )
		.def( "motor_max_torque", &scone::Joint::GetMotorMaxTorque, "Get the max torque [Nm] for the joint motor" )
		.def( "set_motor_target_orientation", &scone::Joint::SetMotorTargetOri, "Set the target orientation of the joint motor" )
		.def( "set_motor_target_velocity", &scone::Joint::SetMotorTargetVel, "Set the target velocity of the joint motor" )
		.def( "add_motor_torque", &scone::Joint::AddMotorTorque, "Add a torque [Nm^3] to this joint motor" )
		.def( "motor_torque", &scone::Joint::GetMotorTorque, "Get the current motor torque [Nm^3]" )
		.def( "set_motor_stiffness", &scone::Joint::SetMotorStiffness, "Set the joint motor stiffness" )
		.def( "set_motor_damping", &scone::Joint::SetMotorDamping, "Set the joint motor damping" )
		;

	py::class_<scone::Dof>( m, "Dof" )
		.def( "name", &scone::Dof::GetName, "Get the name of this Dof" )
		.def( "pos", &scone::Dof::GetPos, "Get the position/value of this Dof [m or rad]" )
		.def( "vel", &scone::Dof::GetVel, "Get the velocity of this Dof" )
		.def( "limit_torque", &scone::Dof::GetLimitMoment, "Get the dof limit torque [Nm]" )
		.def( "set_pos", &scone::Dof::SetPos, "Set the Dof position. This is only applied after Model.init_state_from_dofs()" )
		.def( "set_vel", &scone::Dof::SetVel, "Set the Dof velocity. This is only applied after Model.init_state_from_dofs()" )
		.def( "is_rotational", &scone::Dof::IsRotational, "Check if this is a rotational Dof" )
		.def( "rotation_axis", &scone::Dof::GetRotationAxis, "Get the rotation axis" )
		.def( "is_actuated", &scone::Dof::IsActuated, "Check if this Dof is actuated via a Joint Motor (see Joint)" )
		.def( "actuator_torque", &scone::Dof::GetActuatorTorque, "Get the actuator torque [Nm] applied to this dof" )
		.def( "muscle_moment", &scone::Dof::GetMuscleMoment, "Get the muscle moment [Nm] applied to this dof" )
		;

	py::class_<scone::Actuator>( m, "Actuator" )
		.def( "name", &scone::Actuator::GetName, "Get the name of this Actuator" )
		.def( "input", &scone::Actuator::GetInput, "Get the current Actuator input" )
		.def( "min_input", &scone::Actuator::GetMinInput, "Get the minimum Actuator input value" )
		.def( "max_input", &scone::Actuator::GetMaxInput, "Get the maximum Actuator input value" )
		.def( "add_input", &scone::Actuator::AddInput, "Add input value to this Actuator" )
		.def( "clear_input", &scone::Actuator::ClearInput, "Clear the Actuator input (this is done automatically after each simulation step)" )
		;

	py::class_<scone::Muscle>( m, "Muscle" )
		.def( "name", &scone::Muscle::GetName, "Get the name of this Muscle" )
		.def( "max_isometric_force", &scone::Muscle::GetMaxIsometricForce, "Get the maximum isometric force of this Muscle" )
		.def( "optimal_fiber_length", &scone::Muscle::GetOptimalFiberLength, "Get the optimum fiber lengths of this Muscle" )
		.def( "tendon_slack_length", &scone::Muscle::GetTendonSlackLength, "Get the tendon slack length of this Muscle" )
		.def( "pennation_angle_at_optimal", &scone::Muscle::GetPennationAngleAtOptimal, "Get the pennation angle at optimal of this Muscle" )
		.def( "mass", &scone::Muscle::GetMass, "Get the mass of the Muscle for a specific tension and density" )
		.def( "pcsa", &scone::Muscle::GetMass, "Get the PCSA of the Muscle for a specific tension" )
		.def( "volume", &scone::Muscle::GetMass, "Get the volume of the Muscle for a specific tension" )
		.def( "origin_body", &scone::Muscle::GetOriginBody, py::return_value_policy::reference, "Get the origin Body of this Muscle" )
		.def( "insertion_body", &scone::Muscle::GetInsertionBody, py::return_value_policy::reference, "Get the insertion Body of this Muscle" )
		.def( "joints", &scone::Muscle::GetJoints, py::return_value_policy::reference, "Get the array of Joints this Muscle crosses" )
		.def( "dofs", &scone::Muscle::GetDofs, py::return_value_policy::reference, "Get the array of Dofs this Muscle affects" )
		.def( "dof_moment", &scone::Muscle::GetMoment, "Get the moment of this muscle over a dof" )
		.def( "dof_moment_arm", &scone::Muscle::GetMomentArm, "Get the moment arm of this muscle over a dof" )
		.def( "joint_moment", &scone::Muscle::GetMoment3D, "Get the 3D moment of this muscle over a joint" )
		.def( "joint_moment_arm", &scone::Muscle::GetMomentArm3D, "Get the 3D moment arm of this muscle over a joint" )
		.def( "excitation", &scone::Muscle::GetExcitation, "Get the excitation of this Muscle" )
		.def( "activation", &scone::Muscle::GetActivation, "Get the activation of this Muscle" )
		//.def( "set_excitation", &scone::Muscle::SetExcitation )
		.def( "init_activation", &scone::Muscle::InitializeActivation, "Initialize the Muscle activation" )
		.def( "set_max_activation", &scone::Muscle::SetMaxActivation, "Set the maximum allowed Muscle activation" )
		.def( "set_min_activation", &scone::Muscle::SetMinActivation, "Set the minimum allowed Muscle activation" )
		.def( "force", &scone::Muscle::GetForce, "Get the current force of this Muscle" )
		.def( "force_norm", &scone::Muscle::GetNormalizedForce, "Get the current normalized force this Muscle" )
		.def( "fiber_length", &scone::Muscle::GetFiberLength, "Get the current fiber length of this Muscle" )
		.def( "fiber_length_norm", &scone::Muscle::GetNormalizedFiberLength, "Get the current normalized fiber length this Muscle" )
		.def( "fiber_velocity", &scone::Muscle::GetFiberVelocity, "Get the current fiber velocity of this Muscle" )
		.def( "fiber_velocity_norm", &scone::Muscle::GetNormalizedFiberVelocity, "Get the current normalized fiber velocity this Muscle" )
		;

	py::class_<scone::Leg>( m, "Leg" )
		.def( "name", &scone::Leg::GetName, "Get the name of this Leg" )
		.def( "contact_force", &scone::Leg::GetContactForce, "Get the contact force applied to this Leg" )
		.def( "contact_moment", &scone::Leg::GetContactMoment, "Get the contact moment applied to this Leg" )
		.def( "contact_pos", &scone::Leg::GetContactPos, "Get the position of the contact force applied to this Leg" )
		.def( "contact_load", &scone::Leg::GetLoad, "Get the contact load applied to this Leg" )
		.def( "upper_body", &scone::Leg::GetUpperBody, py::return_value_policy::reference, "Get the upper Body of this Leg" )
		.def( "foot_body", &scone::Leg::GetFootBody, py::return_value_policy::reference, "Get the foot Body of this Leg" )
		.def( "base_body", &scone::Leg::GetBaseBody, py::return_value_policy::reference, "Get the base Body of this Leg" )
		.def( "relative_foot_position", &scone::Leg::GetRelFootPos, "Get the relative foot position [m] of this Leg" )
		.def( "length", &scone::Leg::GetLength, "Get the length [m] of this Leg" )
		;

	py::class_<scone::Measure>( m, "Measure" )
		.def( "name", &scone::Measure::GetName, "Get the name" )
		.def( "final_result", &scone::Measure::GetResult, "Get the final_result" )
		.def( "final_weighted_result", &scone::Measure::GetWeightedResult, "Get the final_weighted_result" )
		.def( "current_result", &scone::Measure::GetCurrentResult, "Get the current_result" )
		.def( "current_weighted_result", &scone::Measure::GetCurrentWeightedResult, "Get the current_weighted_result" )
		;

	py::class_<scone::Model>( m, "Model" )
		.def( "name", &scone::Model::GetName, "Get the name of this Model" )
		.def( "com_pos", &scone::Model::GetComPos, "Get the com_pos [m] of this Model" )
		.def( "com_vel", &scone::Model::GetComVel, "Get the com_vel [m/s] of this Model" )
		.def( "mass", &scone::Model::GetMass, "Get the mass [kg] of this Model" )
		.def( "gravity", &scone::Model::GetGravity, "Get the gravity of this Model" )
		.def( "contact_force", &scone::Model::GetTotalContactForce, "Get the contact force [N] applied to this Model" )
		.def( "contact_load", []( scone::Model& m ) { return m.GetTotalContactForce() / m.GetBW(); }, "Get the contact load [BW] applied to this Model" )
		.def( "contact_power", &scone::Model::GetTotalContactPower, "Get the contact power [W]" )
		.def( "state", &scone::get_state, "Get the current state of this Model" )
		.def( "set_state", &scone::set_state, "Set the current state for this Model" )
		.def( "bodies", []( scone::Model& m ) { return &m.GetBodies(); }, py::return_value_policy::reference, "Get the array of Bodies in this Model" )
		.def( "joints", []( scone::Model& m ) { return &m.GetJoints(); }, py::return_value_policy::reference, "Get the array of Joints in this Model" )
		.def( "dofs", []( scone::Model& m ) { return &m.GetDofs(); }, py::return_value_policy::reference, "Get the array of Dofs in this Model" )
		.def( "actuators", []( scone::Model& m ) { return &m.GetActuators(); }, py::return_value_policy::reference, "Get the array of Actuators in this Model" )
		.def( "muscles", []( scone::Model& m ) { return &m.GetMuscles(); }, py::return_value_policy::reference, "Get the array of Muscles in this Model" )
		.def( "find_body", &scone::Model::FindBody, py::return_value_policy::reference, "Find a body with a specific name" )
		.def( "find_joint", &scone::Model::FindJoint, py::return_value_policy::reference, "Find a joint with a specific name" )
		.def( "find_dof", &scone::Model::FindDof, py::return_value_policy::reference, "Find a dof with a specific name" )
		.def( "find_actuator", &scone::Model::FindActuator, py::return_value_policy::reference, "Find a actuator with a specific name" )
		.def( "find_muscle", &scone::Model::FindMuscle, py::return_value_policy::reference, "Find a muscle with a specific name" )
		.def( "find_muscle_or_group", &scone::Model::FindMuscleOrGroup, py::return_value_policy::reference, "Find a muscle or muscle group with a specific name" )
		.def( "legs", []( scone::Model& m ) { return &m.GetLegs(); }, py::return_value_policy::reference, "Get the array of Legs in this Model" )
		.def( "measure", []( scone::Model& m ) { return m.GetMeasure(); }, py::return_value_policy::reference, "Get the Measure defined inside this Model" )
		.def( "set_dof_positions", &scone::set_dof_positions, "Set the dof positions for this Model (must call init_state_from_dofs() afterwards)" )
		.def( "set_dof_velocities", &scone::set_dof_velocities, "Set the dof velocities for this Model (must call init_state_from_dofs() afterwards)" )
		.def( "init_state_from_dofs", []( scone::Model& m ) { m.InitStateFromDofs(); }, "Initialize the Model state from the current Dof values and equilibrate all muscles (must be called after modifying Dofs)" )
		.def( "adjust_state_for_load", &scone::Model::AdjustStateForLoad, "Adjust the Model state so that is has a specific contact load [BW]" )
		.def( "set_actuator_inputs", &scone::set_actuator_inputs, "Set the actuator inputs for this Model" )
		.def( "actuator_input_array", []( scone::Model& m ) { return scone::get_actuator_inputs( m, g_f32 ); }, "Get an array of current actuator inputs" )
		.def( "dof_position_array", []( scone::Model& m ) { return scone::get_dof_positions( m, g_f32 ); }, "Get an array of current dof positions" )
		.def( "dof_velocity_array", []( scone::Model& m ) { return scone::get_dof_velocities( m, g_f32 ); }, "Get an array of current dof velocities" )
		.def( "muscle_fiber_length_array", []( scone::Model& m ) { return scone::get_muscle_lengths( m, g_f32 ); }, "Get an array of current muscle fiber lengths" )
		.def( "muscle_fiber_velocity_array", []( scone::Model& m ) { return scone::get_muscle_velocities( m, g_f32 ); }, "Get an array of current muscle fiber velocities" )
		.def( "muscle_force_array", []( scone::Model& m ) { return scone::get_muscle_forces( m, g_f32 ); }, "Get an array of current muscle forces" )
		.def( "muscle_activation_array", []( scone::Model& m ) { return scone::get_muscle_activations( m, g_f32 ); }, "Get an array of current muscle activations" )
		.def( "muscle_excitation_array", []( scone::Model& m ) { return scone::get_muscle_excitations( m, g_f32 ); }, "Get an array of current muscle excitations" )
		.def( "delayed_muscle_fiber_length_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::Length, g_f32 ); }, "Get an array of current delayed muscle fiber lengths" )
		.def( "delayed_muscle_fiber_velocity_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::Velocity, g_f32 ); }, "Get an array of current delayed muscle fiber velocities" )
		.def( "delayed_muscle_force_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::Force, g_f32 ); }, "Get an array of current delayed muscle forces" )
		.def( "delayed_dof_position_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::DofPos, g_f32 ); }, "Get an array of current delayed dof positions" )
		.def( "delayed_dof_velocity_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::DofVel, g_f32 ); }, "Get an array of current delayed dof velocities" )
		.def( "delayed_vestibular_array", []( scone::Model& m ) { return get_delayed_sensor_array( m, SensorType::Vestibular, g_f32 ); }, "Get an array of current delayed vestibular sensors" )
		.def( "set_delayed_actuator_inputs", &scone::set_delayed_actuator_inputs, "Set the delayed actuator inputs for this Model" )
		.def( "init_muscle_activations", &scone::init_muscle_activations, "Initialize all muscle activations (must call init_state_from_dofs() afterwards)" )
		.def( "advance_simulation_to", []( scone::Model& m, double t ) { m.AdvanceSimulationTo( t ); }, "Advance the Model simulation to a specific time [s]" )
		.def( "reset", &scone::Model::Reset, "Reset the model to its initial state" )
		.def( "time", &scone::Model::GetTime, "Get the current simulation time [s]" )
		.def( "set_simulation_end_time", &scone::Model::SetSimulationEndTime, "Set the simulation end time [s] for this Model" )
		.def( "has_simulation_ended", &scone::Model::HasSimulationEnded, "Check if the simulation has terminated" )
		.def( "current_measure", &scone::Model::GetCurrentMeasureResult, "Get the current (instantaneous) value of the Measure defined in this Model (useful as reward in Reinforcement Learning)" )
		.def( "final_measure", &scone::Model::GetMeasureResult, "Get the final (aggregate) value of the Measure defined in this Model (useful for shooting-based optimizations)" )
		.def( "log_measure_report", []( scone::Model& m ) { return scone::log_measure_report( m ); }, "Log the Measure report" )
		.def( "set_control_parameter", &scone::set_control_parameter, "Set a control parameter (use get_control_parameter_names() for a list of available control parameters)" )
		.def( "get_control_parameter", &scone::get_control_parameter, "Get the current value of a control parameter" )
		.def( "get_control_parameter_names", &scone::get_control_parameter_names, "Get a list of all control parameter names in this Model" )
		.def( "set_custom_value", &scone::Model::SetCustomValue, "Set a custom value that can be accessed by other components" )
		.def( "get_custom_value", &scone::Model::GetCustomValue, "Get a custom value previously set by set_custom_value or another component" )
		.def( "has_custom_value", &scone::Model::HasCustomValue, "Check if a custom value exists" )
		.def( "get_custom_value_names", &scone::Model::GetCustomValueNames, "Get a list of all custom values in this Model" )
		.def( "get_ray_distance", &scone::get_ray_distance, "Get the distance of a ray cast with a position and direction, up unit max_dist" )
		.def( "integration_step", &scone::Model::GetIntegrationStep, "Get the integration step of this Model" )
		.def( "control_step_size", []( scone::Model& m ) { return m.fixed_control_step_size; }, "Get the control step size [s] of this Model" )
		.def( "set_store_data", &scone::Model::SetStoreData, "Set if data must be stored during Model simulation (slow, do not use in optimizations)" )
		.def( "get_store_data", &scone::Model::GetStoreData, "Get if data is stored during Model simulation" )
		.def( "store_external_data", &scone::Model::StoreExternalData, "Store external data to current data frame" )
		.def( "current_data_frame_time", &scone::Model::GetCurrentDataFrameTime, "Timestamp of most recent SCONE data frame" )
		.def( "set_store_data_profile", &scone::Model::SetStoreDataProfile, "Set store data profile, 0 = default, 1 = minimal" )
		.def( "write_results", &scone::write_results, "Write the simulation results to a .sto file" )
		;

	py::class_<scone::Optimizer>( m, "Optimizer" )
		.def( "run", &scone::Optimizer::Run, "Start the optimization (waits for the optimization to finish)" )
		.def( "run_background", &scone::Optimizer::RunBackground, "Start the optimization in the background (returns immediately)" )
		.def( "output_folder", []( scone::Optimizer& opt ) { return opt.GetOutputFolder().str(); }, "Get the output folder for this optimization" )
		.def( "current_step", &scone::Optimizer::GetCurrentStep, "Get the current optimization step" )
		.def( "fitness", &scone::Optimizer::GetBestFitness, "Get the current best fitness" )
		.def( "wait", &scone::Optimizer::WaitToFinish, "Wait a number milliseconds for the optimization to finish" )
		.def( "finished", &scone::Optimizer::IsFinished, "Check if the optimization has finished" )
		.def( "terminate", &scone::Optimizer::Terminate, "Terminate the optimization" )
		.def( "enable_console_output", &scone::set_optimizer_console_output, "Enable console output" )
		;

	py::class_<scone::sconepy_scenario>( m, "Scenario" )
		.def( "set", &scone::sconepy_scenario::set, "Set a Property in this Scenario" )
		.def( "set_multiple", &scone::sconepy_scenario::set_multiple, "Set multiple Properties in this Scenario" )
		.def( "get", &scone::sconepy_scenario::get, "Get a Property in this Scenario" )
		.def( "create_optimizer", &scone::sconepy_scenario::create_optimizer, "Create an Optimizer from this Scenario" )
		.def( "start_optimization", &scone::sconepy_scenario::start_optimization, "Create and start an Optimizer from this Scenario" )
		;

	m.def( "version", []() { return xo::to_str( scone::GetSconeVersion() ); }, "Get the SCONE version" );
	m.def( "scone_dir", []() { return scone::GetFolder( scone::SconeFolder::Scenarios ).str(); }, "Get the SCONE Scenarios directory" );
	m.def( "scone_results_dir", []() { return scone::GetFolder( scone::SconeFolder::Results ).str(); }, "Get the SCONE Results directory" );
	m.def( "set_log_level", []( int l ) { console_sink.set_log_level( xo::log::level( l ) ); }, "Set the log level" );
	m.def( "evaluate_par_file", &scone::evaluate_par_file, "Evaluate a .par result from a previous optimization" );
	m.def( "load_model", &scone::load_model, py::arg(), py::arg() = std::string(), "Load a .scone Model with an optional .par to initialize the parameters" );
	m.def( "load_scenario", &scone::load_scenario, py::arg(), py::arg() = std::map<std::string, std::string>(), "Load a .scone scenario, with an optional Dict of additional settings" );
	m.def( "is_supported", &scone::is_supported, "Verify if a specific Model type is supported" );
	m.def( "replace_string_tags", &scone::ReplaceStringTags, "Replace the 'DATE_TIME' tag with the current date and time" );

	m.def( "set_array_dtype_float32", []() { g_f32 = true; }, "Set the NumPy floating point type to 32 bit" );
	m.def( "set_array_dtype_float64", []() { g_f32 = false; }, "Set the NumPy floating point type to 64 bit" );
	m.def( "is_array_dtype_float32", []() -> bool { return g_f32; }, "Verify if the NumPy floating point type is 32 bit"  );
	m.def( "is_array_dtype_float64", []() -> bool { return !g_f32; }, "Verify if the NumPy floating point type is 64 bit"  );
}
