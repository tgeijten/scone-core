/*
** GaitAnalysisMeasure.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once
#include "Measure.h"
#include "RangePenalty.h"
#include "scone/core/GaitPlotData.h"
#include "scone/model/Side.h"

namespace scone
{
	/// Measure for locomotion that penalizes deviation from gait analysis data.
	/** Example:
	\verbatim
	GaitAnalysisMeasure {
	}
	\endverbatim
	*/
	class GaitAnalysisMeasure : public Measure
	{
	public:
		GaitAnalysisMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc );

		/// Path of the gait analysis template file to use in this Measure
		path gait_analysis_file;

		/// Load threshold for step detection; default = 0.01.
		Real load_threshold;

		/// Minimum duration of the stance phase [s], shorter contacts are considered as 'bump' during the swing phase; default = 0.1.
		Real min_stance_duration_threshold;

		/// Number of initial gait cycles that are discarded for the final measure; default = 1.
		int skip_cycles;

		/// Fraction of a sample to shift detected contact onset backward in time (0-1); default = 0.5.
		Real contact_timing_offset_;

		/// Channels in template to include (semicolon separated); default = "*"
		String include;

		/// Channels in template to exclude (semicolon separated); default = ""
		String exclude;

		virtual UpdateResult UpdateMeasure( const Model& model, double timestamp ) override;
		virtual double ComputeResult( const Model& model ) override;
		virtual String GetClassSignature() const override;

	private:
		Storage<Real> storage_;
		std::vector<GaitPlotData> plots_;

		struct AnalysisChannel {
			index_t state_idx_;
			index_t storage_idx_;
			index_t plot_idx_;
			Side side_;
			double total_error_ = 0.0;
			size_t cycles_ = 0;
		};
		std::vector<AnalysisChannel> channels_;
	};
}
