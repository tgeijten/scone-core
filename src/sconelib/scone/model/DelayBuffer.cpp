#include "DelayBuffer.h"

#include "scone/core/Exception.h"
#include "xo/container/container_tools.h"
#include "Sensor.h"
#include "Actuator.h"

namespace scone
{
	DelayedSensorValue DelayedSensorGroup::GetDelayedSensorValue( Sensor& sensor, TimeInSeconds delay, TimeInSeconds step_size )
	{
		auto delay_size = GetDelaySampleSize( delay, step_size );
		auto it = xo::find_if( sensors_, [&]( const auto& p ) { return p.first == &sensor; } );
		if ( it != sensors_.end() )
		{
			SCONE_ERROR_IF( delay_size != it->second.delay(), "Sensor " + sensor.GetName() + " cannot have different delay values" );
			return DelayedSensorValue{ it->second };
		}
		else {
			auto& buf = buffers_.try_emplace( delay_size, delay_size, 0 ).first->second;
			auto idx = buf.add_channel();
			sensors_.emplace_back( &sensor, DelayBufferChannel{ &buf, idx } );
			sensors_.back().second.back() = sensors_.back().first->GetValue(); // needed when created 'on-the-fly', e.g. sconepy
			return DelayedSensorValue{ sensors_.back().second };
		}
	}

	void DelayedSensorGroup::AdvanceSensorBuffers()
	{
		for ( auto& b : buffers_ )
			b.second.advance();
	}

	void DelayedSensorGroup::UpdateSensorBufferValues()
	{
		for ( auto& s : sensors_ )
			s.second.back() = s.first->GetValue();
	}

	void DelayedSensorGroup::Reset()
	{
		for ( auto& b : buffers_ )
			b.second.reset();
		UpdateSensorBufferValues();
	}

	DelayedActuatorValue DelayedActuatorGroup::GetDelayedActuatorValue( Actuator& actuator, TimeInSeconds delay, TimeInSeconds step_size )
	{
		auto delay_size = GetDelaySampleSize( delay, step_size );
		auto it = xo::find_if( actuators_, [&]( const auto& p ) { return p.first == &actuator; } );
		if ( it != actuators_.end() )
		{
			SCONE_ERROR_IF( delay_size != it->second.delay(), "Actuator " + actuator.GetName() + " cannot have different delay values" );
			return DelayedActuatorValue{ it->second };
		}
		else {
			auto& buf = buffers_.try_emplace( delay_size, delay_size, 0 ).first->second;
			auto idx = buf.add_channel();
			actuators_.emplace_back( &actuator, DelayBufferChannel{ &buf, idx } );
			return DelayedActuatorValue{ actuators_.back().second };
		}
	}

	void DelayedActuatorGroup::UpdateActuatorInputs()
	{
		for ( auto& a : actuators_ )
			a.first->AddInput( a.second.front() );
	}

	void DelayedActuatorGroup::AdvanceActuatorBuffers()
	{
		for ( auto& b : buffers_ )
			b.second.advance();
	}

	void DelayedActuatorGroup::ClearActuatorBufferValues()
	{
		for ( auto& b : buffers_ )
			b.second.clear_back();
	}

	void DelayedActuatorGroup::Reset()
	{
		for ( auto& b : buffers_ )
			b.second.reset();
	}
}
