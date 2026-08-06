#include "GaitPlotData.h"

#include "xo/container/prop_node_tools.h"
#include "Log.h"

namespace scone
{
	GaitPlotData::GaitPlotData( const PropNode& pn ) :
		INIT_MEMBER( pn, title_, "" ),
		INIT_MEMBER_REQUIRED( pn, left_channel_ ),
		INIT_MEMBER_REQUIRED( pn, right_channel_ ),
		INIT_MEMBER_REQUIRED( pn, row_ ),
		INIT_MEMBER_REQUIRED( pn, column_ ),
		INIT_MEMBER( pn, x_label_, "x" ),
		INIT_MEMBER( pn, y_label_, "y" ),
		INIT_MEMBER( pn, y_min_, 0 ),
		INIT_MEMBER( pn, y_max_, 0 ),
		INIT_MEMBER( pn, channel_offset_, 0 ),
		INIT_MEMBER( pn, channel_multiply_, 1.0 ),
		INIT_MEMBER( pn, norm_offset_, 0 ),
		INIT_MEMBER( pn, mirror_left_, false ),
		norm_event_( pn.try_get<xo::boundsd>( "norm_event" ) ),
		INIT_MEMBER( pn, normalize_norm_data_, false ),
		norm_data_mean_range_{ xo::boundsd::no_bounds() }
	{
		auto* norm_min = pn.try_get_child( "norm_min" );
		auto* norm_max = pn.try_get_child( "norm_max" );
		bool has_min_max = norm_min && norm_max;

		auto* norm_mean = pn.try_get_child( "norm_mean" );
		auto* norm_std = pn.try_get_child( "norm_std" );
		bool has_mean_std = norm_mean && norm_std;

		if ( has_min_max || has_mean_std )
		{
			xo_error_if( has_min_max && norm_min->size() != norm_max->size(), "Mismatch in norm_min and norm_max array length for " + title_ );
			xo_error_if( has_mean_std && norm_mean->size() != norm_std->size(), "Mismatch in norm_mean and norm_std array length for " + title_ );
			auto norm_size = has_min_max ? norm_min->size() : norm_mean->size();
			norm_data_.reserve( norm_size );
			for ( index_t i = 0; i < norm_size; ++i )
			{
				double yt, yb, mean;
				if ( has_min_max ) {
					yt = norm_max->get<double>( i ) + norm_offset_;
					yb = norm_min->get<double>( i ) + norm_offset_;
					mean = ( yt + yb ) / 2;
				} else if ( has_mean_std ) {
					mean = norm_mean->get<double>( i ) + norm_offset_;
					yt = mean + norm_std->get<double>( i );
					yb = mean - norm_std->get<double>( i );
				} else { xo_error( "Unexpected error reading GaitDataPlot" ); }

				y_min_ = xo::min( y_min_, yb );
				y_max_ = xo::max( y_max_, yt );
				norm_data_mean_range_.extend( mean );

				double x = 100.0 * i / ( norm_size - 1 );
				norm_data_.emplace_back( yb, yt );
			}
		}
	}
}
