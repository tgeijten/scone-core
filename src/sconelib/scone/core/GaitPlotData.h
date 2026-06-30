#pragma once

#include "platform.h"
#include "PropNode.h"
#include "types.h"
#include "xo/string/pattern_matcher.h"
#include "xo/container/flat_map.h"
#include "xo/numerical/bounds.h"

namespace scone
{
	struct SCONE_API GaitPlotData
	{
		GaitPlotData( const PropNode& pn );
		~GaitPlotData() = default;

		bool HasNormData() const { return !norm_data_.empty(); }

		String title_;
		xo::pattern_matcher left_channel_;
		xo::pattern_matcher right_channel_;
		int row_;
		int column_;
		String x_label_;
		String y_label_;
		double y_min_;
		double y_max_;

		double channel_offset_;
		double channel_multiply_;
		double norm_offset_;
		bool mirror_left_;

		std::vector<xo::bounds<double>> norm_data_;
		xo::optional<xo::bounds<double>> norm_event_;
	};
}
