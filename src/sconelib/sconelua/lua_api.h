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
#include "ScriptController.h"

#include "xo/geometry/vec3_type.h"
#include "xo/string/string_cast.h"
#include "xo/geometry/quat_type.h"
#include "xo/geometry/quat.h"

namespace sol { class state; }

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

	template< typename T > T& GetRef( T* obj, const std::string& error_msg ) {
		SCONE_ERROR_IF( !obj, error_msg );
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

		/// set a numeric value for channel named key
		void set_value( LuaString key, LuaNumber value ) { frame_[key] = value; }
		/// set a numeric value for channel named key
		void set_vec3( LuaString key, LuaVec3 v ) { string s( key ); frame_[s + "_x"] = v.x; frame_[s + "_y"] = v.y; frame_[s + "_z"] = v.z; }
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
		/// get sum of muscle moments for this dof 
		LuaNumber muscle_moment() { return dof_.GetMuscleMoment(); }

		Dof& dof_;
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
		/// get the passive fiber force [N]
		LuaNumber passive_fiber_force() { return mus_.GetPassiveFiberForce(); }
		/// get the maximum isometric force [N]
		LuaNumber max_isometric_force() { return mus_.GetMaxIsometricForce(); }
		/// get the muscle mass [kg], based on a specific tension of 250000
		LuaNumber mass() { return mus_.GetMass(); }

		Muscle& mus_;
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
		LuaVec3 point_pos( const LuaVec3& p ) { return bod_.GetPosOfPointOnBody( p ); }
		/// get the global linear velocity [m/s] of a local point p on the body
		LuaVec3 point_vel( const LuaVec3& p ) { return bod_.GetLinVelOfPointOnBody( p ); }
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
		void set_com_pos( const LuaVec3& p ) { bod_.SetPos( p ); }
		/// set the orientation of the body
		void set_ori( const LuaQuat& q ) { bod_.SetOrientation( q ); }
		/// set the com velocity [m/s] of the body
		void set_lin_vel( const LuaVec3& v ) { bod_.SetLinVel( v ); }
		/// set the angular velocity [rad/s] of the body
		void set_ang_vel( const LuaVec3& v ) { bod_.SetAngVel( v ); }

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
		void set_motor_target_ori( const LuaQuat& o ) { joint_.SetMotorTargetOri( o ); }
		/// set target velocity of the joint motor
		void set_motor_target_vel( const LuaVec3& v ) { joint_.SetMotorTargetVel( v ); }
		/// set target velocity of the joint motor
		void add_motor_torque( const LuaVec3& v ) { joint_.AddMotorTorque( v ); }
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

		LuaBody parent_body() { return LuaBody( spr_.GetParentBody() ); }
		LuaBody child_body() { return LuaBody( spr_.GetChildBody() ); }
		LuaVec3 pos_in_parent() { return spr_.GetPosInParent(); }
		LuaVec3 pos_in_child() { return spr_.GetPosInChild(); }
		LuaNumber rest_length() { return spr_.GetRestLength(); }
		LuaNumber stiffness() { return spr_.GetStiffness(); }
		LuaNumber damping() { return spr_.GetDamping(); }
		LuaVec3 parent_pos() { return spr_.GetParentPos(); }
		LuaVec3 child_pos() { return spr_.GetChildPos(); }
		bool is_active() { return spr_.IsActive(); }

		void set_parent( LuaBody b, LuaVec3 pos ) { spr_.SetParent( b.bod_, pos ); }
		void set_child( LuaBody b, LuaVec3 pos ) { spr_.SetChild( b.bod_, pos ); }
		void set_rest_length( LuaNumber l ) { spr_.SetRestLength( l ); }
		void set_stiffness( LuaNumber s ) { spr_.SetStiffness( s ); }
		void set_damping( LuaNumber d ) { spr_.SetDamping( d ); }

		Spring& spr_;
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

		Model& mod_;
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

		/// get all available control parameters
		std::vector<std::string> get_control_parameters() { return cont_.GetControlParameters(); }

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
