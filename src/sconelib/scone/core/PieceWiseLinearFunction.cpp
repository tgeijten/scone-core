/*
** PieceWiseLinearFunction.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "PieceWiseLinearFunction.h"
#include "scone/core/string_tools.h"
#include "xo/numerical/math.h"

namespace scone
{
	PieceWiseLinearFunction::PieceWiseLinearFunction( const PropNode& props, Params& par ) :
		control_point_y( props.get_child( "control_point_y" ) ),
		control_point_dt( props.try_get_child( "control_point_dt" ) )
	{
		INIT_PROP( props, control_points, size_t( 0 ) );
		INIT_PROP( props, flat_extrapolation, false );

		for ( index_t cpidx = 0; cpidx < control_points; ++cpidx )
		{
			Real xVal = 0.0;
			if ( cpidx > 0 )
			{
				SCONE_ASSERT_MSG( control_point_dt, "PieceWiseConstantFunction must have control_point_dt when control_points > 1" );
				double dt = par.get( stringf( "DT%d", cpidx - 1 ), *control_point_dt );
				SCONE_ASSERT_MSG( dt > 0.0, "control_point_dt must be > 0" );
				xVal = m_Func.point( cpidx - 1 ).first + dt;
			}
			Real yVal = par.get( stringf( "Y%d", cpidx ), control_point_y );
			m_Func.insert_point( xVal, yVal );
		}
	}

	Real PieceWiseLinearFunction::GetValue( Real x )
	{
		if ( flat_extrapolation && m_Func.size() >= 1 )
			xo::clamp( x, m_Func.front().first, m_Func.back().first );

		return m_Func( x );
	}

	String PieceWiseLinearFunction::GetSignature()
	{
		return stringf( "L%d", m_Func.size() );
	}
}
