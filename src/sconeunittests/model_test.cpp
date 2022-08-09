#include "xo/system/test_case.h"

#include "scone/core/system_tools.h"
#include "scone/model/Model.h"
#include "xo/serialization/prop_node_serializer_zml.h"
#include "scone/core/Factories.h"
#include "scone/core/Log.h"
#include "scone/core/types.h"

namespace scone
{
	std::vector<State> record_states( Model& m, TimeInSeconds end_time = 1.0 ) {
		std::vector<State> s;
		for ( TimeInSeconds t = 0.0; t < end_time; t += 0.01 ) {
			m.AdvanceSimulationTo( t );
			s.emplace_back( m.GetState() );
		}
		return s;
	}

	XO_TEST_CASE( model_test )
	{
		auto modeldir = GetFolder( SCONE_ROOT_FOLDER ) / "resources/unittestdata/models";

		//auto model_pn = xo::load_zml( modeldir / "H0918_hyfydy.scone" );
		auto model_pn = xo::load_zml( modeldir / "H1622_hyfydy.scone" );
		auto model_props = FindFactoryProps( GetModelFactory(), model_pn, "Model" );

		spot::null_objective_info par;
		auto model = CreateModel( model_props, par, modeldir );

		double duration = 1.0;

		State s0_a = model->GetState();
		model->AdvanceSimulationTo( duration );
		auto step_a = model->GetIntegrationStep();
		State s1_a = model->GetState();

		model->ResetState();
		State s0_b = model->GetState();
		model->AdvanceSimulationTo( duration );
		auto step_b = model->GetIntegrationStep();
		State s1_b = model->GetState();

		XO_CHECK( step_a == step_b );
		XO_CHECK( s1_a == s1_b );
		XO_CHECK( s0_a == s0_b );

		std::vector<State> sa, sb;
		model->ResetState();
		sa = record_states( *model );
		model->ResetState();
		sb = record_states( *model );
		XO_CHECK( sa == sb );
	}
}
