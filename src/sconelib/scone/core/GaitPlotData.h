#pragma once

#include "platform.h"
#include "PropNode.h"
#include "types.h"
#include "xo/string/pattern_matcher.h"
#include "xo/container/flat_map.h"
#include "xo/numerical/bounds.h"
#include "scone/model/Side.h"

namespace scone
{
	struct SCONE_API GaitPlotData
	{
		GaitPlotData( const PropNode& pn );
		~GaitPlotData() = default;

		bool HasNormData() const { return !norm_data_.empty(); }
		double GetChannelMultiply( const Side& s ) const { return mirror_left_ && s == Side::Left ? -channel_multiply_ : channel_multiply_; }
		double TransformValue( double v, const Side& s ) const { return channel_offset_ + GetChannelMultiply( s ) * v; }
		bool MustNormalizeNormData() const { return normalize_norm_data_ && HasNormData() && norm_data_mean_range_.upper > 0; }
		double GetNormalizeNormDataFactor( double upper ) const { return MustNormalizeNormData() ? upper / norm_data_mean_range_.upper : 1.0; }

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
		bool normalize_norm_data_;

		// derived variables
		xo::boundsd norm_data_mean_range_;
	};
}

