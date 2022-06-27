#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "scone/core/version.h"
#include "xo/string/string_cast.h"
#include "scone/core/system_tools.h"
#include "xo/system/log_sink.h"
#include "xo/filesystem/path.h"
#include "scone/optimization/opt_tools.h"
#include "scone/sconelib_config.h"
#include "scone/model/Model.h"
#include "spot/par_io.h"
#include "scone/model/Muscle.h"
#include "scone/model/Actuator.h"

namespace py = pybind11;

void evaluate_par_file( const std::string& file ) {
	xo::path scenario_file = scone::FindScenario( file );
	auto scenario_pn = scone::LoadScenario( scenario_file, true ); // for compatibility of versions < 2.0.0
	auto out_path = scenario_file.parent_path();
	scone::log::info( "Evaluating ", file );
	auto results = scone::EvaluateScenario( scenario_pn, file, file );
	scone::log::info( results );
}
scone::ModelUP load_model( const std::string& file ) {
	auto file_path = xo::path( file );
	auto model_pn = load_zml( file_path );
	auto model_props = scone::FindFactoryProps( scone::GetModelFactory(), model_pn, "Model" );
	spot::null_objective_info par;
	return scone::CreateModel( model_props, par, file_path.parent_path() );
}
std::map< std::string, double > get_state( const scone::Model& m ) {
	std::map< std::string, double > dict;
	auto& s = m.GetState();
	for ( size_t i = 0; i < s.GetSize(); ++i )
		dict[ s.GetName( i ) ] = s.GetValue( i );
	return dict;
};
void set_state( scone::Model& m, const std::map< std::string, double >& dict ) {
	scone::State s = m.GetState();
	for ( auto& [key, value] : dict )
		s.SetValue( key, value );
	m.SetState( s, 0.0 );
};
scone::Muscle& get_muscle( const scone::Model& m, size_t i ) { return *m.GetMuscles()[ i ]; }

PYBIND11_MODULE( sconepy, m ) {
	static xo::log::console_sink console_sink( xo::log::level::trace );
	scone::Initialize();

	m.def( "scone_version", []() { return xo::to_str( scone::GetSconeVersion() ); } );
	m.def( "data_dir", []() { return scone::GetDataFolder().str(); } );
	m.def( "set_log_level", []( int l ) { console_sink.set_log_level( xo::log::level( l ) ); } );
	m.def( "evaluate_par_file", &evaluate_par_file );
	m.def( "load_model", &load_model );

	py::class_<scone::Actuator>( m, "Actuator" )
		.def( "name", &scone::Actuator::GetName )
		;

	py::class_<scone::Muscle>( m, "Muscle" )
		.def( "name", &scone::Muscle::GetName )
		.def( "activation", &scone::Muscle::GetActivation )
		.def( "init_activation", &scone::Muscle::InitializeActivation )
		.def( "excitation", &scone::Muscle::GetExcitation )
		.def( "set_excitation", &scone::Muscle::SetExcitation )
		.def( "force", &scone::Muscle::GetForce )
		.def( "normalized_force", &scone::Muscle::GetNormalizedForce )
		.def( "fiber_length", &scone::Muscle::GetFiberLength )
		.def( "normalized_fiber_length", &scone::Muscle::GetNormalizedFiberLength )
		.def( "fiber_velocity", &scone::Muscle::GetFiberVelocity )
		.def( "normalized_fiber_velocity", &scone::Muscle::GetNormalizedFiberVelocity )
		;

	py::class_<scone::Model>( m, "Model" )
		.def( "name", &scone::Model::GetName )
		.def( "advance_simulation_to", &scone::Model::AdvanceSimulationTo )
		.def( "state", &get_state )
		.def( "set_state", &set_state )
		.def( "muscle_count", []( const scone::Model& m ) { return m.GetMuscles().size(); } )
		.def( "muscle", &get_muscle, py::return_value_policy::reference )
		.def( "actuators", []( scone::Model& m ) { return &m.GetActuators(); }, py::return_value_policy::reference )
		.def( "time", &scone::Model::GetTime )
		.def( "set_store_data", &scone::Model::SetStoreData )
		.def( "write_results", []( const scone::Model& m, const std::string& f ) { m.WriteResults( f ); } )
		;
}
