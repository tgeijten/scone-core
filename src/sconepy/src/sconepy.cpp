#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "sconepy_tools.h"

#include "scone/core/version.h"
#include "xo/system/log_sink.h"
#include "scone/sconelib_config.h"
#include "scone/model/Model.h"
#include "scone/model/Muscle.h"
#include "scone/model/Actuator.h"
#include "scone/model/Dof.h"

namespace py = pybind11;

PYBIND11_MODULE( sconepy, m ) {
	static xo::log::console_sink console_sink( xo::log::level::trace );
	scone::Initialize();

	m.def( "scone_version", []() { return xo::to_str( scone::GetSconeVersion() ); } );
	m.def( "data_dir", []() { return scone::GetDataFolder().str(); } );
	m.def( "set_log_level", []( int l ) { console_sink.set_log_level( xo::log::level( l ) ); } );
	m.def( "evaluate_par_file", &scone::evaluate_par_file );
	m.def( "load_model", &scone::load_model );

	py::class_<scone::Vec3>( m, "Vec3" )
		.def_readwrite( "x", &scone::Vec3::x )
		.def_readwrite( "y", &scone::Vec3::y )
		.def_readwrite( "z", &scone::Vec3::z )
		;

	py::class_<scone::Quat>( m, "Quat" )
		.def_readwrite( "w", &scone::Quat::w )
		.def_readwrite( "x", &scone::Quat::x )
		.def_readwrite( "y", &scone::Quat::y )
		.def_readwrite( "z", &scone::Quat::z )
		;

	py::class_<scone::Actuator>( m, "Actuator" )
		.def( "name", &scone::Actuator::GetName )
		;

	py::class_<scone::Muscle>( m, "Muscle" )
		.def( "name", &scone::Muscle::GetName )
		.def( "max_isometric_force", &scone::Muscle::GetMaxIsometricForce )
		.def( "optimal_fiber_length", &scone::Muscle::GetOptimalFiberLength )
		.def( "tendon_slack_length", &scone::Muscle::GetTendonSlackLength )
		.def( "activation", &scone::Muscle::GetActivation )
		.def( "init_activation", &scone::Muscle::InitializeActivation )
		.def( "excitation", &scone::Muscle::GetExcitation )
		.def( "set_excitation", &scone::Muscle::SetExcitation )
		.def( "force", &scone::Muscle::GetForce )
		.def( "force_norm", &scone::Muscle::GetNormalizedForce )
		.def( "fiber_length", &scone::Muscle::GetFiberLength )
		.def( "fiber_length_norm", &scone::Muscle::GetNormalizedFiberLength )
		.def( "fiber_velocity", &scone::Muscle::GetFiberVelocity )
		.def( "fiber_velocity_norm", &scone::Muscle::GetNormalizedFiberVelocity )
		;

	py::class_<scone::Dof>( m, "Dof" )
		.def( "name", &scone::Dof::GetName )
		;

	py::class_<scone::Model>( m, "Model" )
		.def( "name", &scone::Model::GetName )
		.def( "advance_simulation_to", &scone::Model::AdvanceSimulationTo )
		.def( "state", &scone::get_state )
		.def( "set_state", &scone::set_state )
		.def( "com_pos", &scone::Model::GetComPos )
		.def( "com_vel", &scone::Model::GetComVel )
		.def( "muscles", []( scone::Model& m ) { return &m.GetMuscles(); }, py::return_value_policy::reference )
		.def( "muscle_fiber_lengths", []( scone::Model& m ) { return py::array( py::cast( scone::get_muscle_lengths( m ) ) ); } )
		.def( "muscle_fiber_velocities", []( scone::Model& m ) { return py::array( py::cast( scone::get_muscle_velocities( m ) ) ); } )
		.def( "muscle_forces", []( scone::Model& m ) { return py::array( py::cast( scone::get_muscle_forces( m ) ) ); } )
		.def( "muscle_activations", []( scone::Model& m ) { return py::array( py::cast( scone::get_muscle_activations( m ) ) ); } )
		.def( "muscle_excitations", []( scone::Model& m ) { return py::array( py::cast( scone::get_muscle_excitations( m ) ) ); } )
		.def( "actuators", []( scone::Model& m ) { return &m.GetActuators(); }, py::return_value_policy::reference )
		.def( "actuator_inputs", []( scone::Model& m ) { return py::array( py::cast( scone::get_actuator_values( m ) ) ); } )
		.def( "set_actuator_inputs", &scone::set_actuator_values )
		.def( "time", &scone::Model::GetTime )
		.def( "set_store_data", &scone::Model::SetStoreData )
		.def( "write_results", &scone::write_results )
		;
}
