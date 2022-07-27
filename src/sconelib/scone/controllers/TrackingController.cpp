#include "TrackingController.h"

#include "scone/controllers/Controller.h"
#include "scone/model/Actuator.h"
#include "scone/model/Location.h"

#include "scone/core/Factories.h"
#include "scone/core/profiler_config.h"
#include "xo/utility/memoize.h"
#include "scone/core/StorageIo.h"
#include "scone/core/Log.h"

namespace scone
{
	Storage<> read_storage_tmp(const xo::path& f) { Storage<> sto; ReadStorage(sto, f); return sto; }
	static xo::memoize_thread_safe< Storage<>(const xo::path&) > g_storage_cache(read_storage_tmp);

	// some code was copied from MimicMeasure
	TrackingController::TrackingController( const PropNode& props, Params& par, Model& model, const Location& target_area ) :
		Controller( props, par, model, target_area ),
		INIT_MEMBER( props, symmetric, target_area.symmetric_ ),
		INIT_MEMBER( props, include, "*" ),
		INIT_MEMBER( props, exclude, "" ),
		INIT_MEMBER( props, pid, Vec3(1.0,0.0,0.0)),
		INIT_MEMBER( props, scale, 1.0),
		INIT_MEMBER( props, include_states, xo::pattern_matcher("*")),
		INIT_MEMBER( props, exclude_states, xo::pattern_matcher("")),
		INIT_MEMBER( props, time_offset, 0),
		file(FindFile(props.get<path>("file"))),
		storage_(g_storage_cache(file))
	{
		INIT_PROP( props, symmetric, target_area.symmetric_ );
		INIT_PROP( props, min_control_value, xo::constants<Real>::lowest());
		INIT_PROP( props, max_control_value, xo::constants<Real>::max());

		// setup actuator info
		auto incl = xo::pattern_matcher( include );
		auto excl = xo::pattern_matcher( exclude );
		auto& actuators = model.GetActuators();
		for ( size_t idx = 0; idx < actuators.size(); ++idx )
		{
			const auto& name = actuators[ idx ]->GetName();
			if ( incl( name ) && !excl( name ) )
			{
				ActInfo ai;
				ai.full_name = actuators[ idx ]->GetName();
				ai.name = GetNameNoSide( ai.full_name );
				ai.side = GetSideFromName( ai.full_name );
				ai.actuator_idx = idx;

				// see if this actuator is on the right side
				if (target_area.side_ == Side::None || target_area.side_ == ai.side) {
					m_ActInfos.push_back(ai);
					log::info("TrackingController include actuators: " + ai.full_name);
				}
			}
		}

		SCONE_THROW_IF(m_ActInfos.empty(), "No matching actuators");

		SCONE_THROW_IF(storage_.IsEmpty(), file.str() + " contains no data");

		// automatically set stop_time to match data, if not set
		if (stop_time == 0 || stop_time > storage_.Back().GetTime())
			stop_time = storage_.Back().GetTime();

		auto& state = model.GetState();
		for (index_t state_idx = 0; state_idx < state.GetSize(); ++state_idx)
		{
			auto& name = state.GetName(state_idx);
			if (include_states(name) && !exclude_states(name))
			{
				index_t sto_idx = storage_.TryGetChannelIndex(name);
				if (sto_idx != NoIndex)
				{
					auto w = 1.0;
					state_storage_map_.emplace_back(Channel{ state_idx, sto_idx, w });
					channel_errors_.emplace_back(name, 0.0);
					log::info("TrackingController include state channels : " + name);
				}
			}
		}

		log::debug("TrackingController found ", state_storage_map_.size(), " of ", storage_.GetChannelCount(), " channels from ", file);

		SCONE_THROW_IF(state_storage_map_.empty(), "No matching states found in " + file.str());

		model.AddExternalResource(file);

		channel_errors_last_ = channel_errors_;
		channel_errors_sum_  = channel_errors_;

	}

	bool TrackingController::ComputeControls( Model& model, double time )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		double u = 0.0; 
		double dt = model.GetDeltaTime();
		double pe = 0.0, de = 0.0, ie =0.0;

		auto& state = model.GetState();
		index_t error_idx = 0;
		auto frame = storage_.ComputeInterpolatedFrame(time + time_offset);
		for (auto& m : state_storage_map_)
		{
			auto storage_value = frame.value(m.storage_idx_);
			auto e = m.weight_ * (scale * storage_value - state[m.state_idx_]);
			channel_errors_[error_idx].second = e;
			channel_errors_sum_[error_idx].second += e;

			pe += e;
			ie += channel_errors_sum_[error_idx].second;
			if(dt>0.0) de += (e - channel_errors_last_[error_idx].second) / dt;

			error_idx++;
		}

		u = pid[0] * pe + pid[1] * ie + pid[2] * de;

		// apply results to all actuators
		auto& actuators = model.GetActuators();
		for ( ActInfo& ai : m_ActInfos )
		{
			xo::clamp(u, min_control_value, max_control_value);
			// apply results directly to control value
			actuators[ ai.actuator_idx ]->AddInput( u );
		}

		channel_errors_last_ = channel_errors_;

		return false;
	}

	scone::String TrackingController::GetClassSignature() const
	{
		return String("FT");
	}
}
