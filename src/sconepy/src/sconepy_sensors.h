#pragma once

#include "sconepy.h"

#include "xo/xo_types.h"
#include <string>
#include "scone/model/Muscle.h"
#include "scone/model/Dof.h"
#include "scone/model/Sensors.h"
#include "scone/model/MuscleId.h"
#include "sconepy_tools.h"
#include "xo/geometry/quat.h"

enum class SensorType {
	Force,
	Length,
	Velocity,
	DofPos,
	DofVel,
	Vestibular
};

using xo::index_t, xo::no_index;

using DelayedSensorValueVector = std::vector<scone::DelayedSensorValue>;

DelayedSensorValueVector& get_delayed_sensor_value_vector( scone::Model& model, SensorType t ) {
	auto& a = model.GetUserAnyData( g_sensor_user_data_key );
	if ( !a.has_value() )
		a = std::vector<DelayedSensorValueVector>();
	auto& vv = std::any_cast<std::vector<DelayedSensorValueVector>&>( a );
	auto idx = static_cast<size_t>( t );
	if ( idx >= vv.size() )
		vv.resize( idx + 1 );
	return vv.at( idx );
}

template< typename S >
const DelayedSensorValueVector& create_delayed_muscle_sensors( scone::Model& model, SensorType t ) {
	auto& v = get_delayed_sensor_value_vector( model, t );
	SCONE_ASSERT( v.empty() );
	v.reserve( model.GetMuscles().size() );
	for ( auto mus : model.GetMuscles() ) {
		auto delay = model.GetTwoWayNeuralDelay( *mus );
		v.emplace_back( model.GetDelayedSensor( model.AcquireSensor<S>( *mus ), delay ) );
	}
	SCONE_ERROR_IF( v.empty(), "Could not create sensors for type " + xo::to_str( int( t ) ) );
	return v;
}

const DelayedSensorValueVector& create_delayed_dof_sensors( scone::Model& model, SensorType t ) {
	auto& v = get_delayed_sensor_value_vector( model, t );
	SCONE_ASSERT( v.empty() );
	v.reserve( model.GetDofs().size() );
	for ( auto dof : model.GetDofs() ) {
		if ( dof->IsRotational() ) {
			auto delay = model.GetTwoWayNeuralDelay( *dof );
			if ( t == SensorType::DofPos )
				v.emplace_back( model.GetDelayedSensor( model.AcquireSensor<scone::DofPositionSensor>( *dof ), delay ) );
			else if ( t == SensorType::DofVel )
				v.emplace_back( model.GetDelayedSensor( model.AcquireSensor<scone::DofVelocitySensor>( *dof ), delay ) );
		}
	}
	return v;
}

const DelayedSensorValueVector& add_delayed_body_sensor( scone::Model& model, DelayedSensorValueVector& v, const scone::Body& bod ) {
	auto delay = model.GetTwoWayNeuralDelay( bod );
	for ( index_t axis = 0; axis < 3; ++axis ) {
		v.emplace_back( model.GetDelayedSensor( model.AcquireSensor<scone::BodyEulerOriSensor>(
			bod, axis, bod.GetSide() ), delay ) );
	}
	for ( index_t axis = 0; axis < 3; ++axis ) {
		v.emplace_back( model.GetDelayedSensor( model.AcquireSensor<scone::BodyAngularVelocitySensor>(
			bod, scone::Vec3::axis( axis ), scone::GetAxisName( axis ), bod.GetSide() ), delay ) );
	}
	return v;
}

const DelayedSensorValueVector& create_delayed_body_sensors( scone::Model& model, SensorType t, const std::string& bodies ) {
	auto& v = get_delayed_sensor_value_vector( model, t );
	SCONE_ASSERT( v.empty() );
	xo::pattern_matcher body_pat{ bodies };
	for ( auto bod : model.GetBodies() ) {
		if ( body_pat( bod->GetName() ) )
			add_delayed_body_sensor( model, v, *bod );
	}
	return v;
}

const std::string& find_vestibular_body_name( const scone::Model& model ) {
	static const std::vector<std::string> try_names{ "head", "lumbar", "torso", "pelvis" };
	for ( auto& name : try_names )
		if ( scone::HasElementWithName( model.GetBodies(), name ) )
			return name;
	SCONE_ERROR( "Could not find any of the following bodies: " + xo::concatenate_str( try_names, " " ) );
}

const DelayedSensorValueVector& create_delayed_sensors( scone::Model& model, SensorType t ) {
	switch ( t )
	{
	case SensorType::Force:
		return create_delayed_muscle_sensors<scone::MuscleForceSensor>( model, t );
	case SensorType::Length:
		return create_delayed_muscle_sensors<scone::MuscleLengthSensor>( model, t );
	case SensorType::Velocity:
		return create_delayed_muscle_sensors<scone::MuscleVelocitySensor>( model, t );
	case SensorType::DofPos:
	case SensorType::DofVel:
		return create_delayed_dof_sensors( model, t );
	case SensorType::Vestibular:
		return create_delayed_body_sensors( model, t, find_vestibular_body_name( model ) );
	default:
		SCONE_ERROR( "Unsupported sensor type" );
		break;
	}
}

const DelayedSensorValueVector& get_delayed_sensor_values( scone::Model& model, SensorType t ) {
	auto& v = get_delayed_sensor_value_vector( model, t );
	if ( v.empty() ) {
		auto& vnew = create_delayed_sensors( model, t );
		SCONE_ERROR_IF( vnew.empty(), "Could not create sensors for type " + xo::to_str( int( t ) ) );
		return vnew;
	}
	else return v;
}

template< typename T >
py::array_t<T> get_delayed_sensor_array( scone::Model& model, SensorType t ) {
	auto& v = get_delayed_sensor_values( model, t );
	auto [a, p] = make_array<T>( v.size() );
	for ( index_t i = 0; i < v.size(); ++i )
		p[ i ] = static_cast<T>( v[ i ].GetValue() );
	return a;
}

py::array get_delayed_sensor_array( scone::Model& model, SensorType t, bool use_f32 ) {
	if ( use_f32 )
		return get_delayed_sensor_array<float>( model, t );
	else
		return get_delayed_sensor_array<double>( model, t );
}
