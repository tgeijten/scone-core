#pragma once

#include "scone/core/platform.h"
#include "scone/core/types.h"
#include "xo/container/circular_buffer.h"
#include "xo/numerical/math.h"
#include <map>

namespace scone
{
	inline size_t GetDelaySampleSize( TimeInSeconds delay, TimeInSeconds step_size ) {
		return std::max( size_t{ 1 }, xo::round_cast<size_t>( 0.5 * delay / step_size ) );
	}

	using DelayBuffer = xo::circular_buffer<Real>;

	struct DelayBufferChannel {
		DelayBuffer* buffer_ = nullptr;
		index_t channel_ = NoIndex;
		size_t delay() const { return buffer_->capacity(); }
		const Real& front() const { return buffer_->front( channel_ ); }
		Real& back() { return buffer_->back( channel_ ); }
	};

	struct DelayedSensorValue {
		Real GetValue() const { return dbc_.front(); }
		DelayBufferChannel dbc_;
	};

	struct DelayedActuatorValue {
		void AddInput( Real value ) { dbc_.back() += value; }
		DelayBufferChannel dbc_;
	};

	struct SCONE_API DelayedSensorGroup {
		DelayedSensorValue GetDelayedSensorValue( Sensor& sensor, TimeInSeconds delay, TimeInSeconds step_size );
		void AdvanceSensorBuffers();
		void UpdateSensorBufferValues();
		void Reset();

		std::map< size_t, DelayBuffer > buffers_;
		std::vector< std::pair<Sensor*, DelayBufferChannel> > sensors_;
	};
	
	struct SCONE_API DelayedActuatorGroup {
		DelayedActuatorValue GetDelayedActuatorValue( Actuator& actuator, TimeInSeconds delay, TimeInSeconds step_size );
		void UpdateActuatorInputs();
		void AdvanceActuatorBuffers();
		void ClearActuatorBufferValues();
		void Reset();

		std::map< size_t, DelayBuffer > buffers_;
		std::vector< std::pair<Actuator*, DelayBufferChannel> > actuators_;
	};
}
