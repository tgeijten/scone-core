#include "SimulationMeasure.h"
#include "scone/model/Model.h"

namespace scone
{
	SimulationMeasure::SimulationMeasure( const PropNode& pn, Params& par, const Model& model, const Location& loc ) :
		Measure( pn, par, model, loc )
	{}

	bool SimulationMeasure::UpdateMeasure( const Model& model, double timestamp )
	{
		return false;
	}

	double SimulationMeasure::ComputeResult( const Model& model )
	{
		return model.GetSimulationReport()["Simulation Performance"]["force_eval_frequency"].get<double>();
	}
}
