#include <pybind11/pybind11.h>

#include "scone/core/version.h"
#include "xo/string/string_cast.h"
#include "scone/core/system_tools.h"
#include "xo/system/log_sink.h"
#include "xo/filesystem/path.h"
#include "scone/optimization/opt_tools.h"
#include "scone/sconelib_config.h"

namespace py = pybind11;

void evaluate_par_file( const std::string& file ) {
	xo::path scenario_file = scone::FindScenario( file );
	auto scenario_pn = scone::LoadScenario( scenario_file, true ); // for compatibility of versions < 2.0.0
	auto out_path = scenario_file.parent_path();
	scone::log::info( "Evaluating ", file );
	auto results = scone::EvaluateScenario( scenario_pn, file, out_path );
	scone::log::info( results );
}

PYBIND11_MODULE(sconepy, m) {
	static xo::log::console_sink console_sink( xo::log::level::trace );
	scone::Initialize();

    m.attr("__version__") = xo::to_str( scone::GetSconeVersion() );

	m.def( "data_dir", []() { return scone::GetDataFolder().str(); } );
    m.def( "evaluate_par_file", &evaluate_par_file );
}
