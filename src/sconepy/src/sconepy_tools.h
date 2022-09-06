#pragma once

#include "sconepy.h"

#include <string>
#include <filesystem>
#include "xo/filesystem/path.h"
#include "xo/serialization/serialize.h"
#include "xo/serialization/prop_node_serializer_zml.h"
#include "scone/optimization/opt_tools.h"
#include "scone/model/Actuator.h"
#include "scone/core/Factories.h"
#include "spot/par_io.h"
#include "scone/model/Muscle.h"
#include "scone/model/Dof.h"
#include "scone/model/Sensors.h"
#include "scone/model/MuscleId.h"

namespace fs = std::filesystem;
const std::string g_scenario_user_data_key = "SconePy";

template< typename C >
py::array to_array( C&& cont, bool use_f32 ) {
	if ( use_f32 )
		return py::array_t<float>( py::cast( std::move( cont ) ) );
	else
		return py::array_t<double>( py::cast( std::move( cont ) ) );
};

template< typename T >
std::pair<py::array_t<T>, T*> make_array( size_t s ) {
	std::pair<py::array_t<T>, T*> r{ py::array_t<T>( s ), nullptr };
	r.second = static_cast<T*>( r.first.request().ptr );
	return r;
};

namespace scone
{
	void evaluate_par_file( const std::string& file ) {
		xo::path scenario_file = FindScenario( file );
		auto scenario_pn = LoadScenario( scenario_file, true ); // for compatibility of versions < 2.0.0
		auto out_path = scenario_file.parent_path();
		log::info( "Evaluating ", file );
		auto results = EvaluateScenario( scenario_pn, file, file );
		log::info( results );
	}
	ModelUP load_model( const std::string& file ) {
		auto file_path = xo::path( file );
		auto model_pn = xo::load_zml( file_path );
		auto model_props = FindFactoryProps( GetModelFactory(), model_pn, "Model" );
		spot::null_objective_info par;
		auto model = CreateModel( model_props, par, file_path.parent_path() );
		model->GetUserData().add_child( g_scenario_user_data_key, model_pn ); // add model_pn to save config.scone
		//model->AddExternalResource( file_path ); // add .scone file so that it can be copied to results
		return model;
	}
	std::map< std::string, double > get_state( const Model& m ) {
		std::map< std::string, double > dict;
		auto& s = m.GetState();
		for ( size_t i = 0; i < s.GetSize(); ++i )
			dict[ s.GetName( i ) ] = s.GetValue( i );
		return dict;
	};
	void set_state( Model& m, const std::map< std::string, double >& dict ) {
		State s = m.GetState();
		for ( auto& [key, value] : dict )
			s.SetValue( key, value );
		m.SetState( s, 0.0 );
	};

	void check_array_length( size_t expected, size_t received ) {
		SCONE_ERROR_IF( expected != received, stringf( "Invalid array length; expected %d, received %d", expected, received ) );
	}

	void set_dof_positions( Model& m, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		auto& dofs = m.GetDofs();
		check_array_length( dofs.size(), v.shape( 0 ) );
		for ( index_t i = 0; i < dofs.size(); ++i )
			dofs[ i ]->SetPos( v( i ) );
	};

	void set_dof_velocities( Model& m, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		auto& dofs = m.GetDofs();
		check_array_length( dofs.size(), v.shape( 0 ) );
		for ( index_t i = 0; i < dofs.size(); ++i )
			dofs[ i ]->SetVel( v( i ) );
	};

	void init_state_from_dofs( Model& m ) {
		m.UpdateStateFromDofs();
	};

	void init_muscle_activations( Model& m, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		auto mus = m.GetMuscles();
		check_array_length( mus.size(), v.shape( 0 ) );
		for ( index_t i = 0; i < mus.size(); ++i )
			mus[ i ]->InitializeActivation( v( i ) );
	};

	template< typename T, typename C, typename F > py::array_t<T> extract_array( const C& cont, F fn ) {
		auto [v, p] = make_array<T>( std::size( cont ) );
		for ( index_t i = 0; i < std::size( cont ); ++i )
			p[ i ] = static_cast<T>( fn( cont[ i ] ) );
		return v;
	};

	template< typename C, typename F > py::array extract_array( const C& cont, F fn, bool use_f32 ) {
		if ( use_f32 )
			return extract_array<float>( cont, fn );
		else
			return extract_array<double>( cont, fn );
	};

	py::array get_muscle_lengths( const Model& model, bool use_f32 ) {
		return extract_array( model.GetMuscles(), []( const Muscle* m ) { return m->GetNormalizedFiberLength(); }, use_f32 );
	};
	py::array get_muscle_velocities( const Model& model, bool use_f32 ) {
		return extract_array( model.GetMuscles(), []( const Muscle* m ) { return m->GetNormalizedFiberVelocity(); }, use_f32 );
	};
	py::array get_muscle_forces( const Model& model, bool use_f32 ) {
		return extract_array( model.GetMuscles(), []( const Muscle* m ) { return m->GetNormalizedForce(); }, use_f32 );
	};
	py::array get_muscle_activations( const Model& model, bool use_f32 ) {
		return extract_array( model.GetMuscles(), []( const Muscle* m ) { return m->GetActivation(); }, use_f32 );
	};
	py::array get_muscle_excitations( const Model& model, bool use_f32 ) {
		return extract_array( model.GetMuscles(), []( const Muscle* m ) { return m->GetExcitation(); }, use_f32 );
	};
	py::array get_actuator_inputs( const Model& model, bool use_f32 ) {
		return extract_array( model.GetActuators(), []( const Actuator* m ) { return m->GetInput(); }, use_f32 );
	};
	py::array get_dof_positions( const Model& model, bool use_f32 ) {
		return extract_array( model.GetDofs(), []( const Dof* d ) { return d->GetPos(); }, use_f32 );
	};
	py::array get_dof_velocities( const Model& model, bool use_f32 ) {
		return extract_array( model.GetDofs(), []( const Dof* d ) { return d->GetVel(); }, use_f32 );
	};

	template< typename S >
	std::pair<index_t, size_t> create_delayed_muscle_sensors( Model& model ) {
		auto& dsg = model.GetDelayedSensorGroup();
		index_t ofs = dsg.sensors_.size();
		for ( auto mus : model.GetMuscles() ) {
			auto& s = model.AcquireSensor<S>( *mus );
			auto two_way_delay = model.GetTwoWayNeuralDelay( MuscleId( mus->GetName() ).base_ );
			model.GetDelayedSensor( s, two_way_delay );
		}
		size_t size = dsg.sensors_.size() - ofs;
		SCONE_ERROR_IF( size != model.GetMuscles().size(), "Error creating delayed muscle sensors" );
		return { ofs, size };
	}

	template< typename T >
	py::array_t<T> get_delayed_sensor_array( const Model& model, index_t ofs, index_t size ) {
		auto& dsg = model.GetDelayedSensorGroup();
		SCONE_ERROR_IF( ofs + size > dsg.sensors_.size(), "Invalid delayed sensor index" );
		auto [v, p] = make_array<T>( size );
		for ( index_t i = 0; i < size; ++i )
			p[ i ] = static_cast<T>( dsg.sensors_[ ofs + i ].second.front() );
		return v;
	}

	py::array get_delayed_sensor_array( const Model& model, index_t ofs, size_t size, bool use_f32 ) {
		if ( use_f32 )
			return get_delayed_sensor_array<float>( model, ofs, size );
		else
			return get_delayed_sensor_array<double>( model, ofs, size );
	}

	template< typename S >
	py::array get_delayed_sensor_array( Model& model, const String& name, bool use_f32 ) {
		auto& user_pn = model.GetUserData()[ g_scenario_user_data_key ];
		index_t ofs = no_index;
		size_t size = 0;
		if ( auto* pn = model.GetUserData()[ g_scenario_user_data_key ].try_get_child( name ) ) {
			ofs = pn->get<index_t>( 0 );
			size = pn->get<size_t>( 1 );
		}
		else {
			auto& new_pn = user_pn.add_child( name );
			auto p = create_delayed_muscle_sensors<S>( model );
			ofs = p.first;
			size = p.second;
			new_pn.add_value( ofs );
			new_pn.add_value( size );
		}

		if ( use_f32 )
			return get_delayed_sensor_array<float>( model, ofs, size );
		else
			return get_delayed_sensor_array<double>( model, ofs, size );
	}

	py::array get_delayed_muscle_lengths( Model& model, bool use_f32 ) {
		return get_delayed_sensor_array<MuscleLengthSensor>( model, "L", use_f32 );
	}
	py::array get_delayed_muscle_velocities( Model& model, bool use_f32 ) {
		return get_delayed_sensor_array<MuscleVelocitySensor>( model, "V", use_f32 );
	}
	py::array get_delayed_muscle_forces( Model& model, bool use_f32 ) {
		return get_delayed_sensor_array<MuscleForceSensor>( model, "F", use_f32 );
	}

	void set_actuator_values( Model& m, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		auto& act = m.GetActuators();
		check_array_length( act.size(), v.shape( 0 ) );
		for ( index_t i = 0; i < act.size(); ++i ) {
			act[ i ]->ClearInput();
			act[ i ]->AddInput( v( i ) );
		}
	};

	size_t create_delayed_actuators( Model& model ) {
		auto& dag = model.GetDelayedActuatorGroup();
		SCONE_ASSERT( dag.actuators_.empty() );
		for ( auto act : model.GetActuators() ) {
			auto two_way_delay = model.GetTwoWayNeuralDelay( MuscleId( act->GetName() ).base_ );
			model.GetDelayedActuator( *act, two_way_delay );
		}
		return dag.actuators_.size();
	}

	void set_delayed_actuator_values( Model& model, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		auto& dag = model.GetDelayedActuatorGroup();
		if ( dag.actuators_.empty() )
			create_delayed_actuators( model );
		auto& act = dag.actuators_;
		check_array_length( act.size(), v.shape( 0 ) );
		for ( index_t i = 0; i < act.size(); ++i ) {
			act[ i ].first->ClearInput();
			act[ i ].second.back() = v( i );
		}
	};

	py::array set_delayed_actuator_inputs( Model& model, bool use_f32 ) {
		return get_delayed_sensor_array<MuscleForceSensor>( model, "F", use_f32 );
	}

	fs::path to_fs( const xo::path& p ) { return fs::path( p.str() ); }
	xo::path from_fs( const fs::path& p ) { return xo::path( p.string() ); }

	void write_results( const Model& m, std::string dir, std::string filename ) {
		SCONE_ASSERT( !m.GetExternalResources().empty() );
		auto target_dir = to_fs( GetFolder( SCONE_RESULTS_FOLDER ) / dir );
		if ( !fs::exists( target_dir ) )
		{
			// setup output folder
			fs::create_directories( target_dir );
			xo::save_file( m.GetUserData().get_child( g_scenario_user_data_key ), from_fs( target_dir ) / "config.scone" );
			for ( auto& p : m.GetExternalResources() )
				fs::copy( to_fs( p ), target_dir, fs::copy_options::overwrite_existing );
		}
		m.WriteResults( from_fs( target_dir / filename ) );
	};
}
