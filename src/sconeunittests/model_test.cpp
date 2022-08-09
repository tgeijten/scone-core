#include "xo/system/test_case.h"

#include "scone/core/system_tools.h"
#include "scone/model/Model.h"
#include "xo/serialization/prop_node_serializer_zml.h"
#include "scone/core/Factories.h"
#include "scone/core/Log.h"
#include "scone/core/types.h"

namespace scone
{
	std::vector<State> record_states( Model& m, TimeInSeconds end_time = 0.05 ) {
		std::vector<State> s;
		for ( TimeInSeconds t = 0.0; t < end_time; t += 0.01 ) {
			m.AdvanceSimulationTo( t );
			s.emplace_back( m.GetState() );
		}
		return s;
	}

	XO_TEST_CASE_SKIP( model_test )
	{
		auto modeldir = GetFolder( SCONE_ROOT_FOLDER ) / "resources/unittestdata/models";

		auto model_pn = xo::load_zml( modeldir / "H0918_hyfydy.scone" );
		//auto model_pn = xo::load_zml( modeldir / "H1622_hyfydy.scone" );
		auto model_props = FindFactoryProps( GetModelFactory(), model_pn, "Model" );

		spot::null_objective_info par;
		auto model = CreateModel( model_props, par, modeldir );
		const TimeInSeconds end_time = 0.4;
		State sa0 = model->GetState();
		model->AdvanceSimulationTo( end_time );
		State sa1 = model->GetState();
		model->ResetState();
		State sb0 = model->GetState();
		model->AdvanceSimulationTo( end_time );
		State sb1 = model->GetState();
		XO_CHECK( sa0 == sb0 );
		XO_CHECK( sa1 == sb1 );

		//std::vector<State> sa, sb;
		//sa = record_states( *model );
		//model->ResetState();
		//sb = record_states( *model );
		//model->ResetState();
		//sc = record_states( *model );
		//for ( index_t i = 0; i < sa.size(); ++i ) {
		//	log::info( i, ": ", sa[ i ] == sb[ i ] );
		//}
		//XO_CHECK( sa == sb );
	}
}
