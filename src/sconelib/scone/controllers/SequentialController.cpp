/*
** SequentialController.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "SequentialController.h"
#include "xo/string/string_tools.h"
#include "xo/numerical/math.h"
#include "scone/core/string_tools.h"

namespace scone
{
	SequentialController::SequentialController( const PropNode& props, Params& par, Model& model, const Location& loc ) :
		CompositeController( props, par, model, loc ),
		active_idx_( 0 )
	{
		const PropNode& trans_pn = props.get_child( "transition_intervals" );
		SCONE_ERROR_IF( controllers_.empty(), "No Controllers defined in SequentialController" );
		SCONE_ERROR_IF( controllers_.size() - 1 != trans_pn.size(),
			"Wrong number of transition_intervals, expected " + to_str( controllers_.size() - 1 ) );

		start_times_.push_back( 0.0 );
		for ( index_t idx = 0; idx < trans_pn.size(); ++idx ) {
			transition_intervals.push_back( par.get( xo::stringf( "transition%d", idx + 1 ), trans_pn[idx] ) );
			start_times_.push_back( start_times_.back() + transition_intervals.back() );
		}
		SCONE_ASSERT( start_times_.size() == controllers_.size() );
	}

	bool SequentialController::ComputeControls( Model& model, double timestamp )
	{
		active_idx_ = GetActiveIdx( timestamp );
		timestamp -= start_times_[active_idx_];
		return controllers_[active_idx_]->UpdateControls( model, timestamp );
	}

	void SequentialController::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		String name = GetName().empty() ? "SequentialController" : GetName();
		frame[name + ".active_index"] = static_cast<double>( active_idx_ );
		controllers_[active_idx_]->StoreData( frame, flags );
	}

	bool SequentialController::PerformAnalysis( const Model& model, double timestamp )
	{
		auto idx = GetActiveIdx( timestamp );
		timestamp -= start_times_[idx];
		return controllers_[idx]->UpdateAnalysis( model, timestamp );
	}

	xo::index_t SequentialController::GetActiveIdx( double timestamp )
	{
		auto it = std::upper_bound( start_times_.begin(), start_times_.end(), timestamp );
		return index_t( xo::clamped<int>( int( it - start_times_.begin() - 1 ), 0, int( controllers_.size() ) - 1 ) );
	}
}
