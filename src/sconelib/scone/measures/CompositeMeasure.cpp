/*
** CompositeMeasure.cpp
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#include "CompositeMeasure.h"
#include "scone/core/Factories.h"
#include "scone/core/profiler_config.h"
#include "scone/core/Factories.h"
#include "scone/core/Log.h"
#include "xo/container/container_tools.h"
#include "scone/core/string_tools.h"

namespace scone
{
	CompositeMeasure::CompositeMeasure( const PropNode& props, Params& par, const Model& model, const Location& loc ) :
		Measure( props, par, model, loc ),
		dual_sided( props.get_any<bool>( { "dual_sided", "symmetric" }, false ) ), // symmetric is for back. comp.
		INIT_MEMBER( props, use_first_non_zero_result, false )
	{
		auto create_measure = [&]( const FactoryProps& fp ) {
			if ( dual_sided ) {
				m_Measures.push_back( CreateMeasure( fp, par, model, Location( Side::Left, true ) ) );
				m_Measures.push_back( CreateMeasure( fp, par, model, Location( Side::Right, true ) ) );
			}
			else m_Measures.push_back( CreateMeasure( fp, par, model, loc ) );
		};

		// add any Measure
		for ( auto& m : props )
			if ( auto fp = MakeFactoryProps( GetMeasureFactory(), m, "Measure" ) )
				create_measure( fp );

		if ( Measures = props.try_get_child( "Measures" ) )
			for ( auto& m : *Measures )
				if ( auto fp = MakeFactoryProps( GetMeasureFactory(), m, "Measure" ) )
					create_measure( fp );

		// copy minimize flag from first measure
		INIT_PROP( props, minimize, !m_Measures.empty() ? m_Measures.front()->minimize : true );
	}

	void CompositeMeasure::StoreData( Storage< Real >::Frame& frame, const StoreDataFlags& flags ) const
	{
		for ( auto& m : m_Measures )
			m->StoreData( frame, flags );
	}

	UpdateResult CompositeMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		UpdateResult result;
		for ( MeasureUP& m : m_Measures )
			result |= m->UpdateAnalysis( model, timestamp );

		return result;
	}

	double CompositeMeasure::ComputeResult( const Model& model )
	{
		double total = 0.0;
		for ( MeasureUP& m : m_Measures )
		{
			double res_org = m->GetResult( model );
			double res_weighted = m->GetWeightedResult( model );
			if ( use_first_non_zero_result ) {
				if ( total == 0.0 && res_weighted != 0.0 )
					total = res_weighted;
			}
			else total += res_weighted;

			bool hasweight = m->weight != 1;
			bool hasofs = m->result_offset != 0;
			bool hasthreshold = bool( m->threshold );
			string value = stringf( "%g", res_weighted );
			if ( hasweight || hasofs || hasthreshold )
			{
				value += " <- ";
				if ( hasweight )
					value += stringf( "%g * ", m->weight );
				if ( hasofs || hasthreshold )
					value += "(";
				value += stringf( "%g", res_org );
				if ( hasofs )
					value += stringf( " %c %g", m->result_offset > 0 ? '+' : '-', std::abs( m->result_offset ) );
				if ( hasthreshold )
					value += stringf( " > %g", m->threshold );
				if ( hasofs || hasthreshold )
					value += ")";
			}
			report_.add_child( m->GetName(), m->GetReport() ).set_value( value );
		}

		return total;
	}

	double CompositeMeasure::GetCurrentResult( const Model& model )
	{
		double total = 0.0;
		for ( MeasureUP& m : m_Measures )
		{
			auto res_weighted = m->GetCurrentWeightedResult( model );
			if ( use_first_non_zero_result ) {
				if ( res_weighted != 0.0 )
					return res_weighted;
			}
			else total += res_weighted;
		}
		return total;
	}

	void CompositeMeasure::Reset( Model& model )
	{
		Measure::Reset( model );
		for ( auto& c : m_Measures )
			c->Reset( model );
	}

	String CompositeMeasure::GetClassSignature() const
	{
		std::vector< String > strset;
		for ( auto& m : m_Measures )
		{
			string s = m->GetSignature();
			if ( xo::find( strset, s ) == strset.end() )
				strset.emplace_back( s );
		}
		return xo::concat_str( strset, "" );
	}
}
