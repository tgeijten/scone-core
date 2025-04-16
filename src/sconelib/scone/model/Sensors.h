/*
** Sensors.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "Sensor.h"
#include "scone/core/types.h"
#include "scone/core/Vec3.h"
#include "scone/model/Side.h"
#include "xo/numerical/bounds.h"
#include "scone/core/Quat.h"

#if defined(_MSC_VER)
#	pragma warning( push )
#	pragma warning( disable: 4275 )
#endif

namespace scone
{
	// Base class for muscle sensors
	struct SCONE_API MuscleSensor : public Sensor
	{
		MuscleSensor( const Muscle& m ) : muscle_( m ) {}
		const Muscle& muscle_;
	};

	// Sensor for normalized muscle force
	struct SCONE_API MuscleForceSensor : public MuscleSensor
	{
		MuscleForceSensor( const Muscle& m ) : MuscleSensor( m ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	// Sensor for normalized muscle length
	struct SCONE_API MuscleLengthSensor : public MuscleSensor
	{
		MuscleLengthSensor( const Muscle& m ) : MuscleSensor( m ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	// Sensor for normalized muscle lengthening speed
	struct SCONE_API MuscleVelocitySensor : public MuscleSensor
	{
		MuscleVelocitySensor( const Muscle& m ) : MuscleSensor( m ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	// Sensor for normalized muscle length
	struct SCONE_API MuscleLengthVelocitySensor : public MuscleSensor
	{
		MuscleLengthVelocitySensor( const Muscle& m, double kv ) : MuscleSensor( m ), kv_( kv ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
		double kv_;
	};

	// Sensor for normalized muscle length
	struct SCONE_API MuscleLengthVelocitySqrtSensor : public MuscleSensor
	{
		MuscleLengthVelocitySqrtSensor( const Muscle& m, double kv ) : MuscleSensor( m ), kv_( kv ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
		double kv_;
	};

	// Sensor that simulates Ia muscle spindle (based on [Prochazka 1999], p.135)
	struct SCONE_API MuscleSpindleSensor : public MuscleSensor
	{
		MuscleSpindleSensor( const Muscle& m ) : MuscleSensor( m ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	struct SCONE_API MuscleSpindleSensor2 : public MuscleSensor
	{
		MuscleSpindleSensor2( const Muscle& m, double kv, double l0 ) : MuscleSensor( m ), kv_( kv ), l0_( l0 ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
		double kv_, l0_;
	};

	struct SCONE_API MuscleExcitationSensor : public MuscleSensor
	{
		MuscleExcitationSensor( const Muscle& m ) : MuscleSensor( m ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	struct SCONE_API MuscleActivationSensor : public MuscleSensor
	{
		MuscleActivationSensor( const Muscle& m ) : MuscleSensor( m ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	// Base struct for dof sensors
	struct SCONE_API DofSensor : public Sensor
	{
		DofSensor( const Dof& dof, const Dof* root_dof ) : dof_( dof ), root_dof_( root_dof ) {}
		const Dof& dof_;
		const Dof* root_dof_;
	};

	struct SCONE_API DofPositionSensor : public DofSensor
	{
		DofPositionSensor( const Dof& dof, const Dof* root_dof = nullptr ) : DofSensor( dof, root_dof ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	struct SCONE_API DofVelocitySensor : public DofSensor
	{
		DofVelocitySensor( const Dof& dof, const Dof* root_dof = nullptr ) : DofSensor( dof, root_dof ) {}

		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	struct SCONE_API DofPosVelSensor : public DofSensor
	{
		DofPosVelSensor( const Dof& dof, double kv, const Dof* root_dof = nullptr, Side side = Side::None ) :
			DofSensor( dof, root_dof ), kv_( kv ), side_( side ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
		double kv_;
		Side side_;
	};

	// Sensor for normalized leg load, based on target_area
	struct SCONE_API LegLoadSensor : public Sensor
	{
		LegLoadSensor( const Leg& leg, Real gain = 1, Real ofs = 0, xo::boundsd range = xo::boundsd::infinite() ) :
			leg_( leg ), gain_( gain ), ofs_( ofs ), range_( range )
		{}

		virtual String GetName() const override;
		virtual Real GetValue() const override;
		const Leg& leg_;
		Real gain_;
		Real ofs_;
		xo::boundsd range_;
	};

	// Base struct for body sensors
	struct SCONE_API BodyPointSensor : public Sensor
	{
		BodyPointSensor( const Body& body, Vec3 ofs, Vec3 dir ) : body_( body ), offset_( ofs ), direction_( dir ) {}
		const Body& body_;
		Vec3 offset_;
		Vec3 direction_;
	};

	struct SCONE_API BodyPointPositionSensor : public BodyPointSensor
	{
		BodyPointPositionSensor( const Body& body, Vec3 ofs, Vec3 dir ) : BodyPointSensor( body, ofs, dir ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	struct SCONE_API BodyPointVelocitySensor : public BodyPointSensor
	{
		BodyPointVelocitySensor( const Body& body, Vec3 ofs, Vec3 dir ) : BodyPointSensor( body, ofs, dir ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	struct SCONE_API BodyPointAccelerationSensor : public BodyPointSensor
	{
		BodyPointAccelerationSensor( const Body& body, Vec3 ofs, Vec3 dir ) : BodyPointSensor( body, ofs, dir ) {}
		virtual String GetName() const override;
		virtual Real GetValue() const override;
	};

	// this sensor does not work in all directions
	struct SCONE_API BodyOrientationSensor : public Sensor
	{
		BodyOrientationSensor( const Body& body, const Vec3& dir, const String& postfix, Side side );
		virtual String GetName() const override { return name_; }
		virtual Real GetValue() const override;
		const Body& body_;
		const Vec3 dir_;
		const String name_;
	};

	struct SCONE_API BodyEulerOriSensor : public Sensor
	{
		BodyEulerOriSensor( const Body& body, index_t axis, Side side );
		virtual String GetName() const override { return name_; }
		virtual Real GetValue() const override;
		const Body& body_;
		index_t axis_;
		Real scale_;
		String name_;
	};

	struct SCONE_API BodyAngularVelocitySensor : public Sensor
	{
		BodyAngularVelocitySensor( const Body& body, const Vec3& dir, const String& postfix, Side side, double scale = 1.0 );
		virtual String GetName() const override { return name_; }
		virtual Real GetValue() const override;
		const Body& body_;
		Vec3 dir_;
		Real scale_;
		String name_;
	};

	struct SCONE_API BodyOriVelSensor : public Sensor
	{
		BodyOriVelSensor( const Body& body, const Vec3& dir, double kv, const String& postfix, Side side, double target = 0.0 );
		virtual String GetName() const override { return name_; }
		virtual Real GetValue() const override;
		const Body& body_;
		const Real kv_;
		const Vec3 dir_;
		const String name_;
		const double target_;
	};

	struct SCONE_API BodyPostureMuscleSensor : public Sensor
	{
		BodyPostureMuscleSensor( const Body& body, const Muscle& muscle, const Joint* joint, const Quat& target_ori, Real kp, Real kv );
		virtual String GetName() const override { return name_; }
		virtual Real GetValue() const override;
		const Body& body_;
		const Muscle& muscle_;
		const Joint* joint_;
		const Quat target_ori_;
		const Real kp_;
		const Real kv_;
		const String name_;
	};

	// sensor of the center of mass wrt base of support
	struct SCONE_API ComBosSensor : public Sensor
	{
		ComBosSensor( const Model& mod, const Vec3& dir, double kv, const String& name, Side side );
		virtual String GetName() const override { return name_; }
		virtual Real GetValue() const override;
		const Model& model_;
		const Real kv_;
		const Vec3 dir_;
		const String name_;
	};

	// sensor of the center of mass wrt pivot point
	struct SCONE_API ComPivotPosSensor : public Sensor
	{
		ComPivotPosSensor( const Model& mod, const Body& pivot_body, const Vec3& dir, Side side );
		virtual String GetName() const override;
		virtual Real GetValue() const override;
		const Model& model_;
		const Body& pivot_body_;
		const Vec3 dir_;
	};
	struct SCONE_API ComPivotVelSensor : public Sensor
	{
		ComPivotVelSensor( const Model& mod, const Body& pivot_body, const Vec3& dir, Side side );
		virtual String GetName() const override;
		virtual Real GetValue() const override;
		const Model& model_;
		const Body& pivot_body_;
		const Vec3 dir_;
	};

	// sensor of the center of mass wrt pivot point
	struct SCONE_API ComSupportPosSensor : public Sensor
	{
		ComSupportPosSensor( const Model& mod, const Vec3& dir, Side side );
		virtual String GetName() const override;
		virtual Real GetValue() const override;
		const Model& model_;
		const Vec3 dir_;
		const Side side_;
	};
	struct SCONE_API ComSupportVelSensor : public Sensor
	{
		ComSupportVelSensor( const Model& mod, const Vec3& dir, Side side );
		virtual String GetName() const override;
		virtual Real GetValue() const override;
		const Model& model_;
		const Vec3 dir_;
		const Side side_;
	};

	struct SCONE_API ModulatedSensor : public Sensor
	{
		ModulatedSensor( const Sensor& sensor, const Sensor& modulator, double gain, double ofs, const String& name, xo::boundsd mod_range = { 0.0, 1.0 } );
		virtual String GetName() const override { return name_; }
		virtual Real GetValue() const override;
		const Sensor& sensor_;
		const Sensor& modulator_;
		double gain_;
		double ofs_;
		String name_;
		xo::boundsd mod_range_;
	};
}

#if defined(_MSC_VER)
#	pragma warning( pop )
#endif
