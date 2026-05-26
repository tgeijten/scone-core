/*
** PieceWiseCubicSpline.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "PieceWiseCubicSpline.h"
#include "scone/core/string_tools.h"
#include "xo/numerical/math.h"

namespace scone
{
	PieceWiseCubicSpline::PieceWiseCubicSpline( const PropNode& props, Params& par ) :
		INIT_MEMBER_REQUIRED( props, control_points ),
		control_point_y( props.get_child( "control_point_y" ) ),
		control_point_dy( props.get_child( "control_point_dy" ) ),
		control_point_dt( props.try_get_child( "control_point_dt" ) ),
		control_point_t0( par.try_get( "T0", props, "control_point_t0", 0.0 ) ),
		INIT_MEMBER( props, flat_extrapolation, false )
	{
		SCONE_CHECK_RANGE( control_points, 1, 100 );

		Real cp_x = control_point_t0;
		for ( index_t cp_idx = 0; cp_idx < control_points; ++cp_idx )
		{
			if ( cp_idx > 0 )
			{
				SCONE_ASSERT_MSG( control_point_dt, "PieceWiseConstantFunction must have control_point_dt when control_points > 1" );
				TimeInSeconds dt = par.get( stringf( "DT%d", cp_idx - 1 ), *control_point_dt );
				SCONE_ASSERT_MSG( dt > 0.0, "control_point_dt must be > 0" );
				cp_x += dt;
			}
			Real cp_y = par.get( stringf( "Y%d", cp_idx ), control_point_y );
			Real cp_dy = par.get( stringf( "DY%d", cp_idx ), control_point_dy );
			m_Func.insert_point( cp_x, cp_y, cp_dy );
		}
	}

	Real PieceWiseCubicSpline::GetValue( Real x )
	{
		if ( flat_extrapolation && m_Func.size() >= 1 )
			xo::clamp( x, m_Func.front().first, m_Func.back().first );

		return m_Func( x );
	}

	String PieceWiseCubicSpline::GetSignature()
	{
		return stringf( "S%d", m_Func.size() );
	}
}
