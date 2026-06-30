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
		norm_event_( pn.try_get<xo::bounds<double>>( "norm_event" ) )
	{
		auto* norm_min = pn.try_get_child( "norm_min" );
		auto* norm_max = pn.try_get_child( "norm_max" );
		if ( norm_min && norm_max )
		{
			if ( norm_min->size() == norm_max->size() )
			{
				norm_data_.reserve( norm_min->size() );
				for ( index_t i = 0; i < norm_min->size(); ++i )
				{
					auto yt = norm_max->get<double>( i ) + norm_offset_;
					auto yb = norm_min->get<double>( i ) + norm_offset_;
					y_min_ = xo::min( y_min_, yb );
					y_max_ = xo::max( y_max_, yt );
					double x = 100.0 * i / ( norm_min->size() - 1 );
					norm_data_.emplace_back( yb, yt );
				}
			} else log::warning( "Invalid norm data for ", title_, ", norm_min has ", norm_min->size(), " data points, norm_max has ", norm_max->size() );
		}
	}
}
