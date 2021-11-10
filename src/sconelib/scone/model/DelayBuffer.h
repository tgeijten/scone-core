#pragma once

#include "scone/core/types.h"
#include <map>
#include "xo/container/circular_buffer.h"

namespace scone
{
	using DelayBuffer = xo::circular_buffer<Real>;

	struct DelayBufferChannel {
		DelayBuffer& buffer_;
		index_t channel_;
		size_t delay() const { return buffer_.capacity(); }
		const Real& front() const { return buffer_.front( channel_ ); }
		Real& back() { return buffer_.back( channel_ ); }
	};

	struct DelayedSensorValue {
		Real GetValue() const { return dbc_.front(); }
		DelayBufferChannel dbc_;
	};

	struct DelayedActuatorValue {
		void AddInput( Real value ) { dbc_.back() += value; }
		DelayBufferChannel dbc_;
	};

	struct DelayedSensorGroup {
		DelayedSensorValue GetDelayedSensorValue( Sensor* sensor, size_t delay );
		void AdvanceSensorBuffers();
		void UpdateSensorBufferValues();

		std::map< size_t, DelayBuffer > buffers_;
		std::vector< std::pair<Sensor*, DelayBufferChannel> > sensors_;
	};
	
	struct DelayedActuatorGroup {
		DelayedActuatorValue GetDelayedActuatorValue( Actuator* actuator, size_t delay );
		void UpdateActuatorInputs();
		void AdvanceActuatorBuffers();
		void ClearActuatorBufferValues();

		std::map< size_t, DelayBuffer > buffers_;
		std::vector< std::pair<Actuator*, DelayBufferChannel> > actuators_;
	};
}
