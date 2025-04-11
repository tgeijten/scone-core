#pragma once

#include "scone/core/types.h"
#include "scone/optimization/Params.h"
#include "scone/model/Model.h"
#include "scone/model/Actuator.h"
#include "scone/model/Muscle.h"
#include "scone/model/Dof.h"
#include "scone/model/Joint.h"
#include "scone/core/Log.h"
#include "scone/core/Storage.h"
#include "scone/controllers/CompositeController.h"
#include "scone/model/SensorDelayAdapter.h"
#include "scone/model/Sensors.h"
#include "ScriptController.h"
#include "xo/geometry/vec3_type.h"
#include "xo/string/string_cast.h"
#include "xo/geometry/quat.h"
#include "sol_config.h"

namespace sol { class state; }

// check a pointer function argument and throw with (somewhat) useful error message
#define LUA_ARG_REF( ptr ) GetArgRef( ptr, __FUNCTION__ )

namespace scone
{
	using LuaString = const char*;
	using LuaNumber = double;

	template< typename T > T& GetByLuaIndex( std::vector<T>& vec, int index ) {
		SCONE_ERROR_IF( index < 1 || index > vec.size(), "Index must be between 1 and " + xo::to_str( vec.size() ) );
		return vec[index - 1];
	}

	template< typename T > T& GetByLuaName( std::vector<T>& vec, const std::string name ) {
		auto it = std::find_if( vec.begin(), vec.end(), [&]( const T& item ) { return item->GetName() == name; } );
		SCONE_ERROR_IF( it == vec.end(), "Could not find \"" + name + "\"" );
		return *it;
	}

	template< typename T > int GetLuaIndex( std::vector<T>& vec, const std::string name ) {
		auto it = std::find_if( vec.begin(), vec.end(), [&]( const T& item ) { return item->GetName() == name; } );
		if ( it != vec.end() )
			return static_cast<int>( it - vec.begin() ) + 1;
		else return 0;
	}

	inline std::string GetLuaMethodName( const char* method_name ) {
		return xo::replace_str( xo::remove_str( method_name, "scone::" ), "::", ":" );
	}

	template< typename T > T& GetRef( T* obj, const char* error_msg ) {
		SCONE_ERROR_IF( !obj, error_msg );
		return *obj;
	}

	template< typename T > T& GetArgRef( T* obj, const char* method_name ) {
		SCONE_ERROR_IF( !obj, GetLuaMethodName( method_name ) + "(): invalid argument" );
		return *obj;
	}

	/// 3d vector type with components x, y, z
	using LuaVec3 = Vec3d;
	using LuaQuat = Quatd;

	/// Access to scone logging and parameters
	/** Use this for logging, or accessing parameters defined in scone. Lua example:
	\verbatim
	scone.debug( 'This is a debug message!' )
	scone.info( 'This is a info message!' )
	scone.warning( 'This is a warning!' )
	scone.error( 'This is an error!' )
	local body_name = scone.body_name -- access parameter defined in ScriptMeasure or ScriptController
	\endverbatim
	*/
	struct LuaScone
	{
		/// display trace message
		static void trace( LuaString msg ) { log::trace( msg ); }
		/// display debug message
		static void debug( LuaString msg ) { log::debug( msg ); }
		/// display info message
		static void info( LuaString msg ) { log::info( msg ); }
		/// display warning message
		static void warning( LuaString msg ) { log::warning( msg ); }
		/// display error message
		static void error( LuaString msg ) { log::error( msg ); }
		/// create quaternion from Euler angles (xyz degrees)
		static LuaQuat quat_from_euler_deg( double x, double y, double z ) {
			return xo::quat_from_euler( xo::degreed( x ), xo::degreed( y ), xo::degreed( z ) );
		}
		/// create quaternion from Euler angles (xyz radians)
		static LuaQuat quat_from_euler_rad( double x, double y, double z ) {
			return xo::quat_from_euler( xo::radiand( x ), xo::radiand( y ), xo::radiand( z ) );
		}
	};

	/// Access to writing data for scone Analysis window
	struct LuaFrame
	{
		LuaFrame( Storage<Real>::Frame& f ) : frame_( f ) {}

		/// set a numeric value for channel named key (is ignored when nil)
		void set_value( LuaString key, sol::optional<LuaNumber> value ) { if ( value ) frame_[key] = *value; }
		/// set a numeric value for channel named key
		void set_vec3( LuaString key, LuaVec3* pv ) { string s( key ); auto& v = LUA_ARG_REF( pv ); frame_[s + "_x"] = v.x; frame_[s + "_y"] = v.y; frame_[s + "_z"] = v.z; }
		/// set a boolean (true or false) value for channel named key
		void set_bool( LuaString key, bool b ) { frame_[key] = b ? 1.0 : 0.0; }
		/// get time of current frame
		LuaNumber time() { return frame_.GetTime(); }

		Storage<Real>::Frame& frame_;
	};

	/// Actuator type for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaActuator
	{
		LuaActuator( Actuator& a ) : act_( a ) {}

		/// get the name of the actuator
		LuaString name() { return act_.GetName().c_str(); }
		/// add a value to the normalized actuator input
		void add_input( LuaNumber value ) { act_.AddInput( value ); }
		/// get the current actuator input
		LuaNumber input() { return act_.GetInput(); }
		/// get minimum allowed value for actuator input
		LuaNumber min_input() { return act_.GetMinInput(); }
		/// get maximum allowed value for actuator input
		LuaNumber max_input() { return act_.GetMaxInput(); }

		Actuator& act_;
	};

	/// Sensor type for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaSensor
	{
		LuaSensor( SensorDelayAdapter& s ) : sensor_( s ) {}

		/// get the delayed value of the sensor
		LuaNumber value( LuaNumber delay ) { return sensor_.GetValue( delay ); }

		SensorDelayAdapter& sensor_;
	};

	/// Sensor with neural delay for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaDelayedSensor
	{
		LuaDelayedSensor( const DelayedSensorValue& s ) : sensor_( s ) {}

		/// get the delayed value of the sensor
		LuaNumber value() { return sensor_.GetValue(); }
		LuaNumber delay_buffer_size() { return static_cast<LuaNumber>( sensor_.dbc_.delay() ); }

		DelayedSensorValue sensor_;
	};

	/// Actuator with neural delay for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaDelayedActuator
	{
		LuaDelayedActuator( const DelayedActuatorValue& a ) : actuator_( a ) {}

		/// get the delayed value of the sensor
		void add_input( LuaNumber value ) { return actuator_.AddInput( value ); }
		LuaNumber delay_buffer_size() { return static_cast<LuaNumber>( actuator_.dbc_.delay() ); }

		DelayedActuatorValue actuator_;
	};

	/// Dof (degree-of-freedom) type for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaDof
	{
		LuaDof( Dof& d ) : dof_( d ) {}

		/// get the name of the muscle
		LuaString name() { return dof_.GetName().c_str(); }
		/// get the current value (position) of the dof in [m] or [rad]
		LuaNumber position() { return dof_.GetPos(); }
		/// get the current velocity of the dof in [m/s] or [rad/s]
		LuaNumber velocity() { return dof_.GetVel(); }
		/// check if this dof is actuated
		bool is_actuated() { return dof_.IsActuated(); }
		/// add a value to the actuator input (only for actuated dofs)
		void add_input( LuaNumber value ) { dof_.AddInput( value ); }
		/// get the current actuator input (only for actuated dofs)
		LuaNumber input() { return dof_.GetInput(); }
		/// get minimum allowed value for actuator input
		LuaNumber min_input() { return dof_.GetMinInput(); }
		/// get maximum allowed value for actuator input
		LuaNumber max_input() { return dof_.GetMaxInput(); }
		/// get lowest (possibly negative) possible actuator torque [Nm] for this dof
		LuaNumber min_torque() { return dof_.GetMinTorque(); }
		/// get highest possible actuator torque [Nm] for this dof
		LuaNumber max_torque() { return dof_.GetMaxTorque(); }
		/// get highest possible actuator torque [Nm] for this dof
		LuaNumber actuator_torque() { return dof_.GetActuatorTorque(); }
		/// get sum of muscle moments for this dof 
		LuaNumber muscle_moment() { return dof_.GetMuscleMoment(); }

		/// create a sensor for delayed dof position
		LuaDelayedSensor create_delayed_position_sensor( LuaNumber delay )
		{ return get_delayed_sensor<DofPositionSensor>( delay ); }
		/// create a sensor for delayed dof velocity
		LuaDelayedSensor create_delayed_velocity_sensor( LuaNumber delay )
		{ return get_delayed_sensor<DofVelocitySensor>( delay ); }

		Dof& dof_;

	private:
		// access non-const model, needed for creating delayed sensors and actuators
		Model& model() { return const_cast<Model&>( dof_.GetModel() ); }
		template<typename T> DelayedSensorValue get_delayed_sensor( LuaNumber delay ) {
			return model().GetDelayedSensor( model().AcquireSensor<T>( dof_ ), 2 * delay );
		}
	};

	/// Body type for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaBody
	{
		LuaBody( Body& b ) : bod_( b ) {}
		LuaBody( const Body& b ) : bod_( const_cast<Body&>( b ) ) {}

		/// get the name of the body
		LuaString name() { return bod_.GetName().c_str(); }
		/// get the mass of the body [kg]
		LuaNumber mass() { return bod_.GetMass(); }
		/// get the diagonal of the inertia tensor of the body
		LuaVec3 inertia_diagonal() { return bod_.GetInertiaTensorDiagonal(); }
		/// get the current com position [m]
		LuaVec3 com_pos() { return bod_.GetComPos(); }
		/// get the current com velocity [m/s]
		LuaVec3 com_vel() { return bod_.GetComVel(); }
		/// get the current com acceleration [m/s%%^%%2]
		LuaVec3 com_acc() { return bod_.GetComAcc(); }
		/// get the global position [m] of a local point p on the body
		LuaVec3 point_pos( const LuaVec3* p ) { return bod_.GetPosOfPointOnBody( LUA_ARG_REF( p ) ); }
		/// get the global linear velocity [m/s] of a local point p on the body
		LuaVec3 point_vel( const LuaVec3* p ) { return bod_.GetLinVelOfPointOnBody( LUA_ARG_REF( p ) ); }
		/// get the body orientation as a quaternion
		LuaQuat ori() { return bod_.GetOrientation(); }
		/// get the body orientation as a 3d rotation vector [rad]
		LuaVec3 ang_pos() { return rotation_vector_from_quat( bod_.GetOrientation() ); }
		/// get the angular velocity [rad/s] of the body
		LuaVec3 ang_vel() { return bod_.GetAngVel(); }
		/// get the angular acceleration [rad/s%%^%%2] of the body
		LuaVec3 ang_acc() { return bod_.GetAngAcc(); }
		/// get the contact force vector [N] applied to this body via contact geometry
		LuaVec3 contact_force() { return bod_.GetContactForce(); }
		/// get the contact moment vector [Nm] applied to this body via contact geometry
		LuaVec3 contact_moment() { return bod_.GetContactMoment(); }
		/// get contact point vector [m] of a contact force applied to this body (zero if no contact)
		LuaVec3 contact_point() { return bod_.GetContactPoint(); }
		/// add external force [N] to body com
		void add_external_force( LuaNumber x, LuaNumber y, LuaNumber z ) { bod_.AddExternalForce( Vec3d( x, y, z ) ); }
		/// add external moment [Nm] to body
		void add_external_moment( LuaNumber x, LuaNumber y, LuaNumber z ) { bod_.AddExternalMoment( Vec3d( x, y, z ) ); }
		/// set the com position [m] of the body
		void set_com_pos( const LuaVec3* p ) { bod_.SetPos( LUA_ARG_REF( p ) ); }
		/// set the orientation of the body
		void set_ori( const LuaQuat* q ) { bod_.SetOrientation( LUA_ARG_REF( q ) ); }
		/// set the com velocity [m/s] of the body
		void set_lin_vel( const LuaVec3* v ) { bod_.SetLinVel( LUA_ARG_REF( v ) ); }
		/// set the angular velocity [rad/s] of the body
		void set_ang_vel( const LuaVec3* v ) { bod_.SetAngVel( LUA_ARG_REF( v ) ); }

		Body& bod_;
	};

	/// Body type for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaJoint
	{
		LuaJoint( Joint& j ) : joint_( j ) {}

		/// get the name of the body
		LuaString name() { return joint_.GetName().c_str(); }
		/// get the current com position [m]
		LuaVec3 pos() { return joint_.GetPos(); }
		/// get the joint reaction force vector [N]
		LuaVec3 reaction_force() { return joint_.GetReactionForce(); }
		/// get the joint limit torque vector [Nm]
		LuaVec3 limit_torque() { return joint_.GetLimitTorque(); }
		/// get the joint limit power [W]
		Real limit_power() { return joint_.GetLimitPower(); }
		/// get the joint load [BW]
		Real load() { return joint_.GetLoad(); }
		/// check if this joint has a motor
		bool has_motor() { return joint_.HasMotor(); }
		/// set target orientation of the joint motor
		void set_motor_target_ori( const LuaQuat* o ) { joint_.SetMotorTargetOri( LUA_ARG_REF( o ) ); }
		/// set target velocity of the joint motor
		void set_motor_target_vel( const LuaVec3* v ) { joint_.SetMotorTargetVel( LUA_ARG_REF( v ) ); }
		/// set target velocity of the joint motor
		void add_motor_torque( const LuaVec3* v ) { joint_.AddMotorTorque( LUA_ARG_REF( v ) ); }
		/// set stiffness of the joint motor
		void set_motor_stiffness( LuaNumber kp ) { joint_.SetMotorStiffness( kp ); }
		/// set damping of the joint motor
		void set_motor_damping( LuaNumber kd ) { joint_.SetMotorDamping( kd ); }

		Joint& joint_;
	};

	/// Spring type for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaSpring
	{
		LuaSpring( Spring& spr ) : spr_( spr ) {}

		/// get current parent body this spring is attached to
		LuaBody parent_body() { return LuaBody( spr_.GetParentBody() ); }
		/// get current child body this spring is attached to
		LuaBody child_body() { return LuaBody( spr_.GetChildBody() ); }
		/// get position in parent body
		LuaVec3 pos_in_parent() { return spr_.GetPosInParent(); }
		/// get position in child body
		LuaVec3 pos_in_child() { return spr_.GetPosInChild(); }
		/// get spring rest length
		LuaNumber rest_length() { return spr_.GetRestLength(); }
		/// get spring stiffness
		LuaNumber stiffness() { return spr_.GetStiffness(); }
		/// get spring damping
		LuaNumber damping() { return spr_.GetDamping(); }
		/// get spring parent pos in world coordinates
		LuaVec3 parent_pos() { return spr_.GetParentPos(); }
		/// get spring child pos in world coordinates
		LuaVec3 child_pos() { return spr_.GetChildPos(); }
		/// check if the spring is active (attached to two bodies)
		bool is_active() { return spr_.IsActive(); }

		/// attach the spring to a local point on a parent body
		void set_parent( LuaBody b, LuaVec3 pos ) { spr_.SetParent( b.bod_, pos ); }
		/// attach the spring to a local point on a child body
		void set_child( LuaBody b, LuaVec3 pos ) { spr_.SetChild( b.bod_, pos ); }
		/// set the spring rest length
		void set_rest_length( LuaNumber l ) { spr_.SetRestLength( l ); }
		/// set the spring stiffness
		void set_stiffness( LuaNumber s ) { spr_.SetStiffness( s ); }
		/// set the spring damping
		void set_damping( LuaNumber d ) { spr_.SetDamping( d ); }

		Spring& spr_;
	};

	/// Muscle type for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaMuscle
	{
		LuaMuscle( Muscle& m ) : mus_( m ) {}

		/// get the name of the muscle
		LuaString name() { return mus_.GetName().c_str(); }
		/// add a value to the normalized actuator input
		void add_input( LuaNumber value ) { mus_.AddInput( value ); }
		/// get the current actuator input
		LuaNumber input() { return mus_.GetInput(); }
		/// get the normalized excitation level [0..1] of the muscle
		LuaNumber excitation() { return mus_.GetExcitation(); }
		/// get the normalized activation level [0..1] of the muscle
		LuaNumber activation() { return mus_.GetActivation(); }
		/// get the fiber length [m] of the contractile element
		LuaNumber fiber_length() { return mus_.GetFiberLength(); }
		/// get the normalized fiber length of the contractile element
		LuaNumber normalized_fiber_length() { return mus_.GetNormalizedFiberLength(); }
		/// get the optimal fiber length [m]
		LuaNumber optimal_fiber_length() { return mus_.GetOptimalFiberLength(); }
		/// get the pennation angle at optimal fiber length [rad]
		LuaNumber pennation_angle_at_optimal() { return mus_.GetPennationAngleAtOptimal(); }
		/// get the fiber lengthening velocity [m/s]
		LuaNumber fiber_velocity() { return mus_.GetFiberVelocity(); }
		/// get the normalized fiber lengthening velocity [m/s]
		LuaNumber normalized_fiber_velocity() { return mus_.GetNormalizedFiberVelocity(); }
		/// get the maximum fiber contraction velocity [m/s]
		LuaNumber max_contraction_velocity() { return mus_.GetMaxContractionVelocity(); }
		/// get the tendon length [m]
		LuaNumber tendon_length() { return mus_.GetTendonLength(); }
		/// get the normalized fiber length of the contractile element
		LuaNumber normalized_tendon_length() { return mus_.GetNormalizedTendonLength(); }
		/// get the optimal fiber length [m]
		LuaNumber tendon_slack_length() { return mus_.GetTendonSlackLength(); }
		/// get the muscle-tendon-unit length [m]
		LuaNumber muscle_tendon_length() { return mus_.GetLength(); }
		/// get the muscle-tendon-unit lengthening velocity [m/s]
		LuaNumber muscle_tendon_velocity() { return mus_.GetVelocity(); }
		/// get the current muscle force [N]
		LuaNumber force() { return mus_.GetForce(); }
		/// get the normalized muscle force [0..1]
		LuaNumber normalized_force() { return mus_.GetNormalizedForce(); }
		/// get the active fiber force [N]
		LuaNumber active_fiber_force() { return mus_.GetActiveFiberForce(); }
		/// get the active fiber force-length multiplier
		LuaNumber active_force_length_multiplier() { return mus_.GetActiveForceLengthMultipler(); }
		/// get the active force-velocity multiplier
		LuaNumber force_velocity_multiplier() { return mus_.GetForceVelocityMultipler(); }
		/// get the passive fiber force [N]
		LuaNumber passive_fiber_force() { return mus_.GetPassiveFiberForce(); }
		/// get the cosine of the pennation angle
		LuaNumber cos_pennation_angle() { return mus_.GetCosPennationAngle(); }
		/// get the maximum isometric force [N]
		LuaNumber max_isometric_force() { return mus_.GetMaxIsometricForce(); }
		/// get the muscle mass [kg], based on a specific tension of 250000
		LuaNumber mass() { return mus_.GetMass(); }
		/// get the 3D moment arm [Nm3] for a specific joint
		LuaNumber moment_arm( const LuaDof* dof ) { return mus_.GetMomentArm( LUA_ARG_REF( dof ).dof_ ); }
		/// get the 3D moment arm [Nm3] for a specific joint
		LuaVec3 moment_arm_3d( const LuaJoint* joint ) { return mus_.GetMomentArm3D( LUA_ARG_REF( joint ).joint_ ); }

		/// create a sensor for delayed muscle force
		LuaDelayedSensor create_delayed_force_sensor( LuaNumber delay )
		{ return get_delayed_sensor<MuscleForceSensor>( delay ); }
		/// create a sensor for delayed muscle length
		LuaDelayedSensor create_delayed_length_sensor( LuaNumber delay )
		{ return get_delayed_sensor<MuscleLengthSensor>( delay ); }
		/// create a sensor for delayed muscle velocity
		LuaDelayedSensor create_delayed_velocity_sensor( LuaNumber delay )
		{ return get_delayed_sensor<MuscleVelocitySensor>( delay ); }
		/// create a sensor for delayed muscle activation
		LuaDelayedSensor create_delayed_activation_sensor( LuaNumber delay )
		{ return get_delayed_sensor<MuscleActivationSensor>( delay ); }

		/// create an actuator with neural delay
		LuaDelayedActuator create_delayed_actuator( LuaNumber delay )
		{ return model().GetDelayedActuator( mus_, 2 * delay ); }

		Muscle& mus_;

	private:
		// access non-const model, needed for creating delayed sensors and actuators
		Model& model() { return const_cast<Model&>( mus_.GetModel() ); }
		template<typename T> DelayedSensorValue get_delayed_sensor( LuaNumber delay ) {
			return model().GetDelayedSensor( model().AcquireSensor<T>( mus_ ), 2 * delay );
		}
	};

	/// Model type for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaModel
	{
		LuaModel( Model& m ) : mod_( m ) {}

		/// get the current simulation time [s]
		LuaNumber time() { return mod_.GetTime(); }
		/// get the previous simulation delta time [s]
		LuaNumber delta_time() { return mod_.GetDeltaTime(); }
		/// get the max_duration of the simulation objective
		LuaNumber max_duration() { return mod_.GetSimulationEndTime(); }
		/// get the current com position [m]
		LuaVec3 com_pos() { return mod_.GetComPos(); }
		/// get the current com velocity [m/s]
		LuaVec3 com_vel() { return mod_.GetComVel(); }
		/// get the model mass [kg]
		LuaNumber mass() { return mod_.GetMass(); }
		/// get the gravitational pull [m/s^2]
		LuaVec3 gravity() { return mod_.GetGravity(); }
		/// get the gravitational pull [m/s^2]
		void set_gravity( LuaVec3 g ) { return mod_.SetGravity( g ); }

		/// get the actuator at index (starting at 1)
		LuaActuator actuator( int index ) { return *GetByLuaIndex( mod_.GetActuators(), index ); }
		/// find an actuator with a specific name
		LuaActuator find_actuator( LuaString name ) { return *GetByLuaName( mod_.GetActuators(), name ); }
		/// number of actuators
		int actuator_count() { return static_cast<int>( mod_.GetActuators().size() ); }

		/// get the muscle at index (starting at 1)
		LuaDof dof( int index ) { return *GetByLuaIndex( mod_.GetDofs(), index ); }
		/// find a muscle with a specific name
		LuaDof find_dof( LuaString name ) { return *GetByLuaName( mod_.GetDofs(), name ); }
		/// number of dofs
		int dof_count() { return static_cast<int>( mod_.GetDofs().size() ); }

		/// get the muscle at index (starting at 1)
		LuaMuscle muscle( int index ) { return *GetByLuaIndex( mod_.GetMuscles(), index ); }
		/// find a muscle with a specific name
		LuaMuscle find_muscle( LuaString name ) { return *GetByLuaName( mod_.GetMuscles(), name ); }
		/// number of muscles
		int muscle_count() { return static_cast<int>( mod_.GetMuscles().size() ); }

		/// get the body at index (starting at 1)
		LuaBody body( int index ) { return *GetByLuaIndex( mod_.GetBodies(), index ); }
		/// find a body with a specific name
		LuaBody find_body( LuaString name ) { return *GetByLuaName( mod_.GetBodies(), name ); }
		/// find a body index with a specific name
		int find_body_index( LuaString name ) { return GetLuaIndex( mod_.GetBodies(), name ); }
		/// number of bodies
		int body_count() { return static_cast<int>( mod_.GetBodies().size() ); }
		/// get the ground (static) body
		LuaBody ground_body() { return mod_.GetGroundBody(); }

		/// get the joint at index (starting at 1)
		LuaJoint joint( int index ) { return *GetByLuaIndex( mod_.GetJoints(), index ); }
		/// find a joint with a specific name
		LuaJoint find_joint( LuaString name ) { return *GetByLuaName( mod_.GetJoints(), name ); }
		/// number of joints
		int joint_count() { return static_cast<int>( mod_.GetJoints().size() ); }

		/// get the model interaction spring
		LuaSpring get_interaction_spring() { return GetRef( mod_.GetInteractionSpring(), "Model has no interaction spring" ); }

		/// store a custom value that can be accessed in other scripts
		void set_custom_value( LuaString name, LuaNumber value ) { mod_.GetUserValue( name ) = value; }
		/// retrieve a specific custom value that has previously been set
		LuaNumber get_custom_value( LuaString name ) { return mod_.GetUserValue( name ); }
		/// check a specific custom value exists
		bool has_custom_value( LuaString name ) { return mod_.HasUserValue( name ); }

		/// get two-way neural delay from the neural_delays section in the model, returns zero if not found 
		LuaNumber find_two_way_neural_delay( LuaString name ) { return mod_.TryGetTwoWayNeuralDelay( name ); }
		/// get one-way neural delay from the neural_delays section in the model, returns zero if not found 
		LuaNumber find_one_way_neural_delay( LuaString name ) { return 0.5 * mod_.TryGetTwoWayNeuralDelay( name ); }

		/// create a sensor for delayed muscle force
		LuaSensor create_muscle_force_sensor( LuaString name ) {
			return AcquireMuscleSensorDelayAdapter<MuscleForceSensor>( name ); }
		/// create a sensor for delayed muscle length
		LuaSensor create_muscle_length_sensor( LuaString name ) {
			return AcquireMuscleSensorDelayAdapter<MuscleLengthSensor>( name ); }
		/// create a sensor for delayed muscle velocity
		LuaSensor create_muscle_velocity_sensor( LuaString name ) {
			return AcquireMuscleSensorDelayAdapter<MuscleVelocitySensor>( name ); }
		/// create a sensor for delayed muscle activation
		LuaSensor create_muscle_activation_sensor( LuaString name ) {
			return AcquireMuscleSensorDelayAdapter<MuscleActivationSensor>( name ); }

		/// create a sensor for delayed muscle force
		LuaDelayedSensor create_delayed_muscle_force_sensor( LuaString muscle, LuaNumber delay ) {
			return GetDelayedMuscleSensor<MuscleForceSensor>( muscle, delay ); }
		/// create a sensor for delayed muscle length
		LuaDelayedSensor create_delayed_muscle_length_sensor( LuaString muscle, LuaNumber delay ) {
			return GetDelayedMuscleSensor<MuscleLengthSensor>( muscle, delay ); }
		/// create a sensor for delayed muscle velocity
		LuaDelayedSensor create_delayed_muscle_velocity_sensor( LuaString muscle, LuaNumber delay ) {
			return GetDelayedMuscleSensor<MuscleVelocitySensor>( muscle, delay ); }
		/// create a sensor for delayed muscle activation
		LuaDelayedSensor create_delayed_muscle_activation_sensor( LuaString muscle, LuaNumber delay ) {
			return GetDelayedMuscleSensor<MuscleActivationSensor>( muscle, delay ); }

		/// create an actuator with neural delay
		LuaDelayedActuator create_delayed_muscle_actuator( LuaString muscle, LuaNumber delay ) {
			return GetDelayedMuscleActuator<MuscleActivationSensor>( muscle, delay ); }

		Model& mod_;

	private:

		Muscle& FindMuscle( LuaString name ) { return *GetByLuaName( mod_.GetMuscles(), name ); }
		template<typename T> SensorDelayAdapter& AcquireMuscleSensorDelayAdapter( LuaString name ) {
			return mod_.AcquireDelayedSensor<T>( FindMuscle( name ) );
		}
		template<typename T> DelayedSensorValue GetDelayedMuscleSensor( LuaString name, LuaNumber delay ) {
			return mod_.GetDelayedSensor( mod_.AcquireSensor<T>( FindMuscle( name ) ), 2 * delay );
		}
		template<typename T> DelayedActuatorValue GetDelayedMuscleActuator( LuaString name, LuaNumber delay ) {
			return mod_.GetDelayedActuator( FindMuscle( name ), 2 * delay );
		}
	};

	/// Controller type for use in lua scripting.
	/** The LuaController can be used to control any SCONE Controller that is defined inside the ScriptController.
	For example, if the ScriptController is defined as follows:

	\verbatim
	ScriptController {
		script_file = my_script.lua

		ReflexController { name = Reflex1 ... }
		ReflexController { name = Reflex2 ... }
	}
	\endverbatim

	The ReflexController instances can now be enabled and disabled in the update function of ''my_script.lua'':
	\verbatim
	function update( model, time, controller )
		controller:set_child_enabled(1, true) -- enable first ReflexController
		controller:set_child_enabled(2, true) -- disable second ReflexController

		return false -- return true to terminate the simulation early
	end
	\endverbatim
	*/
	struct LuaController
	{
		LuaController( Controller& c ) : cont_( c ), comp_cont_( dynamic_cast<CompositeController*>( &c ) ) {}

		/// get the name of the body
		LuaString name() { return cont_.GetName().c_str(); }

		/// number of child controllers
		int child_count() { return static_cast<int>( comp_cont_ ? children().size() : 0 ); }

		/// get child controller
		LuaController child( int index ) { return *GetByLuaIndex( children(), index ); }

		/// enable controller 
		void enable( bool enabled ) { cont_.SetDisabled( !enabled ); }

		/// check if controller at index is enabled (starting at 1)
		bool enabled() { return !cont_.IsDisabled(); }

		/// enable controller at index (starting at 1)
		void set_child_enabled( int index, bool enabled ) { GetByLuaIndex( children(), index )->SetDisabled( !enabled ); }

		/// check if controller at index is enabled (starting at 1)
		bool is_child_enabled( int index ) { return !GetByLuaIndex( children(), index )->IsDisabled(); }

		/// set the value of a control parameter, returns the number of parameters that were set
		int set_control_parameter( LuaString name, LuaNumber value ) { return cont_.TrySetControlParameter( name, value ); }

		/// set the value of a control parameter, returns the number of parameters that were set
		LuaNumber get_control_parameter( LuaString name ) {
			if ( auto v = cont_.TryGetControlParameter( name ) )
				return *v;
			else SCONE_ERROR( "Could not find control parameter: " + String( name ) );
		}

		/// get the names of all available control parameters
		std::vector<std::string> get_control_parameter_names() { return cont_.GetControlParameters(); }

		/// set the value of a control parameter, returns the number of parameters that were set
		bool create_control_parameter( LuaString name, LuaNumber value ) {
			if ( auto* sc = dynamic_cast<ScriptController*>( &cont_ ) )
				return sc->CreateControlParameter( name, value );
			else SCONE_ERROR( "Control parameters can only be created in the parent controller" );
		}

		Controller& cont_;
		CompositeController* comp_cont_;

	private:
		std::vector< ControllerUP >& children() { SCONE_ERROR_IF( !comp_cont_, "Controller has no children" ); return comp_cont_->GetChildren(); }
	};

	/// parameter access for use in lua scripting.
	/// See ScriptController and ScriptMeasure for details on scripting.
	struct LuaParams
	{
		LuaParams( Params& p ) : par_( p ) {}

		/// get or create an optimization parameter with a specific name, mean, stdev, minval and maxval
		LuaNumber create_from_mean_std( LuaString name, LuaNumber mean, LuaNumber stdev, LuaNumber minval, LuaNumber maxval ) {
			return par_.get( name, spot::par_t( mean ), spot::par_t( stdev ), spot::par_t( minval ), spot::par_t( maxval ) );
		}
		/// get or create an optimization parameter from a string
		LuaNumber create_from_string( LuaString name, const std::string& value ) {
			return par_.get( name, xo::to_prop_node( value ) );
		}
		/// get a parameter value, returns 0.0 if not exist
		LuaNumber get( LuaString name ) {
			if ( auto p = par_.try_get( name ); p )
				return *p;
			else SCONE_ERROR( "Could not find parameter: " + string( name ) );
		}

		Params& par_;
	};

	void register_lua_wrappers( sol::state& lua );
}
