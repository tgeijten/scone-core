/*
** PieceWiseConstantFunction.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "PieceWiseConstantFunction.h"
#include "scone/core/string_tools.h"
#include "xo/numerical/piecewise_constant_function.h"
#include "Exception.h"

namespace scone
{
	PieceWiseConstantFunction::PieceWiseConstantFunction( const PropNode& props, Params& par ) :
		INIT_MEMBER_REQUIRED( props, control_points ),
		control_point_y( props.get_child( "control_point_y" ) ),
		control_point_dt( props.try_get_child( "control_point_dt" ) )
	{
		SCONE_CHECK_RANGE( control_points, 1, 100 );

		Real cp_x = 0.0;
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
			m_Func.insert_point( cp_x, cp_y );
		}
	}

	Real PieceWiseConstantFunction::GetValue( Real x )
	{
		return m_Func( x );
	}

	String PieceWiseConstantFunction::GetSignature()
	{
		return stringf( "C%d", m_Func.size() );
	}
}
