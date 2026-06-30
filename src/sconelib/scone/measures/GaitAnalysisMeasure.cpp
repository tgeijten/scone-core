/*
** GaitAnalysisMeasure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "GaitAnalysisMeasure.h"
#include "scone/model/Model.h"
#include "scone/core/Exception.h"
#include "scone/core/profiler_config.h"
#include "scone/core/GaitCycle.h"
#include "scone/core/system_tools.h"
#include "xo/serialization/serialize.h"
#include "scone/core/IncludeExcludePattern.h"
#include "xo/container/container_tools.h"

namespace scone
{
	GaitAnalysisMeasure::GaitAnalysisMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
		Measure( props, par, model, loc ),
		gait_analysis_file( FindFile( props.get<path>( "gait_analysis_file" ) ) ),
		INIT_MEMBER( props, load_threshold, 0.01 ),
		INIT_MEMBER( props, min_stance_duration_threshold, 0.1 ),
		INIT_MEMBER( props, skip_cycles, 2 ),
		INIT_MEMBER( props, include, "*" ),
		INIT_MEMBER( props, exclude, "" )
	{
		xo::error_code ec;
		auto plot_pn = xo::load_file( gait_analysis_file, &ec );
		if ( ec.good() ) {
			plots_.reserve( plot_pn.size() );
			for ( const auto& pn : plot_pn )
				plots_.emplace_back( pn.second );
		} else log::error( "Error loading gait analysis template: ", ec.message() );

		model.AddExternalResource( gait_analysis_file );
	}

	UpdateResult GaitAnalysisMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );
		const auto& state = model.GetState();

		// initialize storage
		if ( storage_.IsEmpty() ) {
			size_t s = size_t( 0.5 + model.GetSimulationEndTime() / model.fixed_measure_step_size );
			storage_.Reserve( s );

			// first six channels are grf and cop
			for ( const auto& leg : model.GetLegs() ) {
				storage_.AddChannels( leg.GetName() + ".grf_norm", { "_x", "_y", "_z" } );
				storage_.AddChannels( leg.GetName() + ".cop", { "_x", "_y", "_z" } );
			}

			// add channels from template
			auto match = IncludeExcludePattern{ include, exclude };
			auto& labels = storage_.GetLabels();
			const auto& state_names = model.GetState().GetNames();
			for ( const auto& plot : plots_ ) {
				if ( !plot.HasNormData() )
					continue;
				auto plot_idx = xo::index_of( plot, plots_ );
				for ( Side side : { Side::Left, Side::Right } ) {
					// find plot channel in existing data (i.e. grf)
					auto& channel_pattern = side == Side::Left ? plot.left_channel_ : plot.right_channel_;
					auto storage_idx = xo::find_index_if( labels, [&]( auto& l ) { return channel_pattern( l ); } );
					if ( storage_idx != no_index && match( labels[storage_idx] ) )
						channels_.push_back( AnalysisChannel{ no_index, storage_idx, plot_idx, side } );

					if ( storage_idx != no_index )
						continue;

					// find plot channel in states
					auto state_idx = xo::find_index_if( state_names, [&]( auto& l ) { return channel_pattern( l ); } );
					if ( state_idx != no_index && match( state_names[ state_idx ] ) ) {
						auto sto_idx = storage_.AddChannel( state.GetName( state_idx ) );
						channels_.push_back( AnalysisChannel{ state_idx, sto_idx, plot_idx, side } );
					}
				}
			}
		}

		// store grf and cop channels
		auto& frame = storage_.AddFrame( timestamp );
		for ( index_t idx = 0; idx < model.GetLegCount(); ++idx ) {
			const auto& leg = model.GetLeg( idx );
			auto fv = leg.GetContactForceValue();
			Vec3 grf = fv.force / model.GetBW();
			frame.SetVec3( idx * 6, grf );
			frame.SetVec3( idx * 6 + 3, fv.point );
		}

		// store state channels
		for ( const auto& ch : channels_ ) {
			if ( ch.state_idx_ != no_index )
				frame[ch.storage_idx_] = state.GetValue( ch.state_idx_ );
		}

		return false;
	}

	double GaitAnalysisMeasure::ComputeResult( const Model& model )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		GaitCycleExtractionSettings cfg{ load_threshold, min_stance_duration_threshold };
		auto cycles = ExtractGaitCycles( storage_, cfg );

		for ( index_t cycle_idx = skip_cycles; cycle_idx < cycles.size(); ++cycle_idx ) {
			const auto& cycle = cycles[cycle_idx];
			for ( auto& ch : channels_ ) {
				if ( ch.side_ != cycle.side_ )
					continue;
				const auto& plot = plots_[ch.plot_idx_];
				double lookahead = 0.5 * cycle.duration() / plot.norm_data_.size();
				double factor = plot.mirror_left_ && cycle.side_ == Side::Left ? -plot.channel_multiply_ : plot.channel_multiply_;
				double error = 0.0;
				for ( const auto& r : plot.norm_data_ ) {
					double x = 1.0 * xo::index_of( r, plot.norm_data_ ) / ( plot.norm_data_.size() - 1 );
					auto f = storage_.ComputeInterpolatedFrame( cycle.begin_ + x * cycle.duration() - lookahead );
					auto value = plot.channel_offset_ + factor * f.value( ch.storage_idx_ );
					error += xo::abs( r.get_excess( value ) ) / xo::max( 0.01, r.length() );
				}
				error /= plot.norm_data_.size();
				ch.total_error_ += error;
				ch.cycles_++;
			}
		}

		double penalty = 0.0;
		for ( auto& ch : channels_ ) {
			const auto& name = storage_.GetLabels()[ch.storage_idx_];
			if ( ch.cycles_ == 0 )
				continue;

			auto avg_error = ch.total_error_ / ch.cycles_;
			report_.set( name, avg_error );
			//log::debug( name, "\tc=", ch.cycles_, "\te=", avg_error );
			penalty += avg_error;
		}

		return penalty;
	}

	String GaitAnalysisMeasure::GetClassSignature() const
	{
		return stringf( "GA" );
	}
}

