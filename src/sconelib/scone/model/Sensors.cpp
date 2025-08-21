/*
** Sensors.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "Sensors.h"
#include "Model.h"
#include "Muscle.h"
#include "Body.h"
#include "Dof.h"
#include "Joint.h"
#include "xo/geometry/vec3.h"
#include "xo/geometry/quat.h"
#include "xo/container/container_algorithms.h"
#include "scone/core/string_tools.h"
#include "scone/core/Log.h"
#include "model_tools.h"
#include <numeric>

namespace scone
{
	String MuscleForceSensor::GetName() const { return muscle_.GetName() + ".F"; }
	Real MuscleForceSensor::GetValue() const { return muscle_.GetNormalizedForce(); }

	String MuscleLengthSensor::GetName() const { return muscle_.GetName() + ".L"; }
	Real MuscleLengthSensor::GetValue() const { return muscle_.GetNormalizedFiberLength(); }

	String MuscleVelocitySensor::GetName() const { return muscle_.GetName() + ".V"; }
	Real MuscleVelocitySensor::GetValue() const { return muscle_.GetNormalizedFiberVelocity(); }

	String MuscleLengthVelocitySensor::GetName() const { return muscle_.GetName() + ".L"; }
	Real MuscleLengthVelocitySensor::GetValue() const { return muscle_.GetNormalizedFiberLength() + kv_ * muscle_.GetNormalizedFiberVelocity(); }

	String MuscleLengthVelocitySqrtSensor::GetName() const { return muscle_.GetName() + ".L"; }
	Real MuscleLengthVelocitySqrtSensor::GetValue() const { return muscle_.GetNormalizedFiberLength() + kv_ * xo::signed_sqrt( muscle_.GetNormalizedFiberVelocity() ); }

	String MuscleSpindleSensor::GetName() const { return muscle_.GetName() + ".S"; }
	Real MuscleSpindleSensor::GetValue() const { return muscle_.GetNormalizedSpindleRate(); }

	String MuscleSpindleSensor2::GetName() const { return muscle_.GetName() + ".L"; }
	Real MuscleSpindleSensor2::GetValue() const {
		auto l = muscle_.GetNormalizedFiberLength() - l0_;
		auto v = kv_ * xo::signed_sqrt( muscle_.GetNormalizedFiberVelocity() );
		return std::max( 0.0, l + v );
	}

	String MuscleExcitationSensor::GetName() const { return muscle_.GetName() + ".excitation"; }
	Real MuscleExcitationSensor::GetValue() const { return muscle_.GetExcitation(); }

	String MuscleActivationSensor::GetName() const { return muscle_.GetName() + ".A"; }
	Real MuscleActivationSensor::GetValue() const { return muscle_.GetActivation(); }

	String LegLoadSensor::GetName() const { return leg_.GetName() + ".LD"; }
	Real LegLoadSensor::GetValue() const { return range_.clamped( gain_ * leg_.GetLoad() + ofs_ ); }

	DofSensor::DofSensor( const Dof& dof, const Dof* root_dof ) :
		dof_( dof ), root_dof_( root_dof ),
		root_sign_( root_dof && ( dof.GetSide() == Side::Left && root_dof->GetSide() == Side::None ) ? -1.0 : 1.0 )
	{
		if ( root_dof_ )
			log::trace( dof.GetName(), " root=", root_dof_->GetName(), " root_sign=", root_sign_ );
	}

	String DofSensor::GetDofName() const
	{
		return root_dof_ ? root_dof_->GetName() + "_" + dof_.GetName() : dof_.GetName();
	}

	String DofPositionSensor::GetName() const { return GetDofName() + ".DP"; }
	Real DofPositionSensor::GetValue() const {
		return root_dof_ ? root_sign_ * root_dof_->GetPos() + dof_.GetPos() : dof_.GetPos();
	}

	String DofVelocitySensor::GetName() const { return GetDofName() + ".DV"; }
	Real DofVelocitySensor::GetValue() const {
		return root_dof_ ? root_sign_ * root_dof_->GetVel() + dof_.GetVel() : dof_.GetVel();
	}

	String DofPosVelSensor::GetName() const { return GetSidedName( dof_.GetName(), side_ ) + ".DPV"; }
	Real DofPosVelSensor::GetValue() const {
		Real value = root_dof_ ? root_sign_ * root_dof_->GetPos() + dof_.GetPos() + kv_ * ( root_sign_ * root_dof_->GetVel() + dof_.GetVel() ) :
			dof_.GetPos() + kv_ * dof_.GetVel();
		return side_ == Side::Right ? -value : value; // mirror for right side, see SensorNeuron.cpp
	}

	String BodyPointPositionSensor::GetName() const { return body_.GetName() + ".PP"; }
	Real BodyPointPositionSensor::GetValue() const {
		return xo::dot_product( direction_, body_.GetPosOfPointOnBody( offset_ ) );
	}

	String BodyPointVelocitySensor::GetName() const { return body_.GetName() + ".PV"; }
	Real BodyPointVelocitySensor::GetValue() const {
		return xo::dot_product( direction_, body_.GetLinVelOfPointOnBody( offset_ ) );
	}

	String BodyPointAccelerationSensor::GetName() const { return body_.GetName() + ".PA"; }
	Real BodyPointAccelerationSensor::GetValue() const {
		return xo::dot_product( direction_, body_.GetLinAccOfPointOnBody( offset_ ) );
	}

	BodyOrientationSensor::BodyOrientationSensor( const Body& body, const Vec3& dir, const String& postfix, Side side ) :
		body_( body ), dir_( GetSidedAxis( dir, side ) ), name_( GetSidedName( body_.GetName() + postfix, side ) + ".BO" ) {}
	Real BodyOrientationSensor::GetValue() const {
		return xo::dot_product( body_.GetOrientation() * dir_, xo::rotation_vector_from_quat( xo::normalized_fast( body_.GetOrientation() ) ) );
	}

	inline Side GetSensorSide( const Body& b, Side s ) {
		if ( b.GetModel().scone_version >= xo::version( 2, 0, 6 ) )
			return GetSideOr( b.GetSide(), s ); // only return side if body has no side
		else return s; // legacy behavior: always return side, leads to issues with contralateral bodies
	}

	BodyEulerOriSensor::BodyEulerOriSensor( const Body& body, index_t axis, Side side ) :
		body_( body ),
		axis_( axis ),
		scale_( GetSidedAxisScale( axis, GetSensorSide( body, side ) ) ),
		name_( GetSidedNameIfUnsided( body_.GetName(), side ) + ".BO" + GetAxisName( axis ) )
	{
		SCONE_ERROR_IF( axis_ >= 3, "Invalid axis: " + to_str( axis_ ) );
	}
	Real BodyEulerOriSensor::GetValue() const {
		return scale_ * xo::euler_yzx_from_quat( xo::normalized_fast( body_.GetOrientation() ) )[axis_].value;
	}

	BodyAngularVelocitySensor::BodyAngularVelocitySensor( const Body& body, const Vec3& dir, const String& postfix, Side side, double scale ) :
		body_( body ),
		dir_( GetSidedAxis( dir, GetSensorSide( body, side ) ) ),
		scale_( scale ),
		name_( GetSidedNameIfUnsided( body_.GetName(), side ) + ".BAV" + postfix )
	{}
	Real BodyAngularVelocitySensor::GetValue() const {
		return scale_ * xo::dot_product( body_.GetOrientation() * dir_, body_.GetAngVel() );
	}

	BodyOriVelSensor::BodyOriVelSensor( const Body& body, const Vec3& dir, double kv, const String& postfix, Side side, double target ) :
		body_( body ),
		kv_( kv ),
		dir_( GetSidedAxis( dir, side ) ),
		name_( GetSidedName( body_.GetName() + postfix, side ) + ".BOV" ),
		target_( target )
	{}
	Real BodyOriVelSensor::GetValue() const {
		auto ori_rv = xo::rotation_vector_from_quat( xo::normalized( body_.GetOrientation() ) );
		auto dir = body_.GetOrientation() * dir_;
		return xo::dot_product( dir, ori_rv ) + kv_ * xo::dot_product( dir, body_.GetAngVel() ) - target_;
	}

	BodyPostureMuscleSensor::BodyPostureMuscleSensor( const Body& body, const Muscle& muscle, const Joint* joint, const Quat& target_ori, Real kp, Real kv ) :
		body_( body ),
		muscle_( muscle ),
		joint_( joint ),
		target_ori_( target_ori ),
		kp_( kp ),
		kv_( kv ),
		name_( body_.GetName() + '-' + muscle_.GetName() + ".POS" )
	{
		if ( !joint_ ) {
			const auto& rb = body_.GetRealBody();
			for ( const auto* j : muscle_.GetJoints() )
				if ( &j->GetParentBody() == &rb || &j->GetBody() == &rb ) { joint_ = j; break; }
		}
		SCONE_ERROR_IF( !joint_, muscle_.GetName() + " does not cross joints on " + body_.GetName() );
	}

	Real BodyPostureMuscleSensor::GetValue() const {
		auto rot = xo::rotation_vector_from_quat( -body_.GetOrientation() * target_ori_ );
		auto ang_vel = body_.GetAngVel();
		auto mom = xo::normalized( muscle_.GetMomentArm3D( *joint_ ) );
		auto p = kp_ * xo::dot_product( rot, mom ) - kv_ * xo::dot_product( ang_vel, mom );
		return p;
	}

	ComBosSensor::ComBosSensor( const Model& mod, const Vec3& dir, double kv, const String& name, Side side ) :
		model_( mod ),
		kv_( kv ),
		dir_( GetSidedDirection( dir, side ) ),
		name_( GetSidedName( name, side ) + ".CB" )
	{
		SCONE_ERROR_IF( model_.GetLegCount() <= 0, "Could not find legs in model" );
		SCONE_ERROR_IF( !model_.HasRootBody(), "Model has no root body" );
	}
	Real ComBosSensor::GetValue() const {
		const auto com = model_.GetProjectedOntoGround( model_.GetComPos() + kv_ * model_.GetComVel() );
		const auto bosp = xo::average( model_.GetLegs(), Vec3(),
			[]( const Vec3& v, const Leg& l ) { return v + l.GetFootBody().GetComPos(); } );
		const auto bosv = xo::average( model_.GetLegs(), Vec3(),
			[]( const Vec3& v, const Leg& l ) { return v + l.GetFootBody().GetComVel(); } );
		const auto bos = model_.GetProjectedOntoGround( bosp + kv_ * bosv );
		return xo::dot_product( model_.GetRootBody().GetOrientation() * dir_, com - bos );
	}

	ComPivotPosSensor::ComPivotPosSensor( const Model& mod, const Body& pivot_body, const Vec3& dir, Side side ) :
		model_( mod ), pivot_body_( pivot_body ), dir_( GetSidedDirection( dir, side ) ) {
		SCONE_ERROR_IF( !model_.HasRootBody(), "Model has no root body" );
	}
	String ComPivotPosSensor::GetName() const {
		return "com-" + pivot_body_.GetName() + ".CP" + GetDominantComponentName( dir_ );
	}
	Real ComPivotPosSensor::GetValue() const {
		auto dir = normalized( projected_xz( model_.GetRootBody().GetOrientation() * dir_ ) );
		return xo::dot_product( dir, model_.GetComPos() - pivot_body_.GetComPos() );
	}

	ComPivotVelSensor::ComPivotVelSensor( const Model& mod, const Body& pivot_body, const Vec3& dir, Side side ) :
		model_( mod ), pivot_body_( pivot_body ), dir_( GetSidedDirection( dir, side ) ) {
		SCONE_ERROR_IF( !model_.HasRootBody(), "Model has no root body" );
	}
	String ComPivotVelSensor::GetName() const {
		return "com-" + pivot_body_.GetName() + ".CV" + GetDominantComponentName( dir_ );
	}
	Real ComPivotVelSensor::GetValue() const {
		auto dir = normalized( projected_xz( model_.GetRootBody().GetOrientation() * dir_ ) );
		return xo::dot_product( dir, model_.GetComVel() - pivot_body_.GetComVel() );
	}

	ComSupportPosSensor::ComSupportPosSensor( const Model& mod, const Vec3& dir, Side side ) :
		model_( mod ), dir_( GetSidedDirection( dir, side ) ), side_( side ) {
		SCONE_ERROR_IF( !model_.HasRootBody(), "Model has no root body" );
	}
	String ComSupportPosSensor::GetName() const {
		return GetSidedName( "com_support", side_ ) + ".CSP" + GetDominantComponentName( dir_ );
	}
	Real ComSupportPosSensor::GetValue() const {
		auto dir = normalized( projected_xz( model_.GetRootBody().GetOrientation() * dir_ ) );
		auto support_pos = xo::average( model_.GetLegs(), Vec3(),
			[]( const Vec3& v, const Leg& l ) { return v + l.GetFootBody().GetComPos(); } );
		return xo::dot_product( dir, model_.GetComPos() - support_pos );
	}

	ComSupportVelSensor::ComSupportVelSensor( const Model& mod, const Vec3& dir, Side side ) :
		model_( mod ), dir_( GetSidedDirection( dir, side ) ), side_( side ) {
		SCONE_ERROR_IF( !model_.HasRootBody(), "Model has no root body" );
	}
	String ComSupportVelSensor::GetName() const {
		return GetSidedName( "com_support", side_ ) + ".CSV" + GetDominantComponentName( dir_ );
	}
	Real ComSupportVelSensor::GetValue() const {
		auto dir = normalized( projected_xz( model_.GetRootBody().GetOrientation() * dir_ ) );
		auto support_vel = xo::average( model_.GetLegs(), Vec3(),
			[]( const Vec3& v, const Leg& l ) { return v + l.GetFootBody().GetComVel(); } );
		return xo::dot_product( dir, model_.GetComVel() - support_vel );
	}

	ModulatedSensor::ModulatedSensor( const Sensor& sensor, const Sensor& modulator, double gain, double ofs, const String& name, xo::boundsd mod_range ) :
		sensor_( sensor ), modulator_( modulator ), gain_( gain ), ofs_( ofs ), name_( name ), mod_range_( mod_range ) {}
	Real ModulatedSensor::GetValue() const {
		auto mv = mod_range_.clamped( gain_ * modulator_.GetValue() + ofs_ );
		return sensor_.GetValue() * mv;
	}
}
