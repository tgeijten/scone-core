#pragma once

#include "sconepy.h"

#include "xo/xo_types.h"
#include <string>
#include "scone/model/Muscle.h"
#include "scone/model/Dof.h"
#include "scone/model/Sensors.h"
#include "scone/model/MuscleId.h"
#include "sconepy_tools.h"

enum class SensorType {
	Force,
	Length,
	Velocity,
	DofPos,
	DofVel,
	Vestibular
};

using xo::index_t, xo::no_index;

static const std::string g_sensor_user_data_key = "SPDS";

template< typename S >
std::pair<index_t, size_t> create_delayed_muscle_sensors( scone::Model& model ) {
	auto& dsg = model.GetDelayedSensorGroup();
	index_t ofs = dsg.sensors_.size();
	for ( auto mus : model.GetMuscles() ) {
		auto& s = model.AcquireSensor<S>( *mus );
		auto two_way_delay = model.GetTwoWayNeuralDelay( scone::MuscleId( mus->GetName() ).base_ );
		model.GetDelayedSensor( s, two_way_delay );
	}
	size_t size = dsg.sensors_.size() - ofs;
	SCONE_ERROR_IF( size != model.GetMuscles().size(), "Error creating delayed muscle sensors" );
	return { ofs, size };
}


std::pair<index_t, size_t> create_delayed_sensors( scone::Model& model, SensorType t ) {
	switch ( t )
	{
	case SensorType::Force: return create_delayed_muscle_sensors<scone::MuscleForceSensor>( model );
	case SensorType::Length: return create_delayed_muscle_sensors<scone::MuscleLengthSensor>( model );
	case SensorType::Velocity: return create_delayed_muscle_sensors<scone::MuscleVelocitySensor>( model );
	case SensorType::DofPos:
	case SensorType::DofVel:
	case SensorType::Vestibular:
	default:
		SCONE_ERROR( "Unsupported sensor type" );
		break;
	}
}

template< typename T >
py::array_t<T> get_delayed_sensor_array( const scone::Model& model, index_t ofs, index_t size ) {
	auto& dsg = model.GetDelayedSensorGroup();
	SCONE_ERROR_IF( ofs + size > dsg.sensors_.size(), "Invalid delayed sensor index" );
	auto [v, p] = make_array<T>( size );
	for ( index_t i = 0; i < size; ++i )
		p[ i ] = static_cast<T>( dsg.sensors_[ ofs + i ].second.front() );
	return v;
}

py::array get_delayed_sensor_array( const scone::Model& model, index_t ofs, size_t size, bool use_f32 ) {
	if ( use_f32 )
		return get_delayed_sensor_array<float>( model, ofs, size );
	else
		return get_delayed_sensor_array<double>( model, ofs, size );
}

py::array get_delayed_sensor_array( scone::Model& model, SensorType t, bool use_f32 ) {
	auto& sensor_info_pn = model.GetUserData()[ g_sensor_user_data_key ];
	index_t sensor_info_idx = static_cast<index_t>( t ) * 2;
	if ( sensor_info_pn.size() < sensor_info_idx + 2 )
		sensor_info_pn.resize( sensor_info_idx + 2, no_index );
	index_t ofs = sensor_info_pn.get<index_t>( sensor_info_idx );
	size_t size = sensor_info_pn.get<index_t>( sensor_info_idx + 1 );

	if ( ofs == no_index ) {
		std::tie( ofs, size ) = create_delayed_sensors( model, t );
		sensor_info_pn[ sensor_info_idx ] = ofs;
		sensor_info_pn[ sensor_info_idx + 1 ] = size;
	}

	return get_delayed_sensor_array( model, ofs, size, use_f32 );
}
