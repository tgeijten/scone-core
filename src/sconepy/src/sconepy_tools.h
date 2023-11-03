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
#include "scone/controllers/ExternalController.h"

namespace fs = std::filesystem;

namespace scone
{
	void evaluate_par_file( const std::string& file ) {
		auto scenario_pn = LoadScenario( file );
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
		if ( !model->GetController() ) {
			PropNode cpn{ { "ExternalController", PropNode() } };
			auto fp = FindFactoryProps( GetControllerFactory(), cpn, "Controller" );
			spot::null_objective_info par;
			model->CreateController( fp, par );
		}
		return model;
	}
	bool is_supported( const std::string& type_id ) {
		return GetModelFactory().has_type( type_id );
	}
	std::map< std::string, double > get_state( const Model& model ) {
		std::map< std::string, double > dict;
		auto& s = model.GetState();
		for ( size_t i = 0; i < s.GetSize(); ++i )
			dict[s.GetName( i )] = s.GetValue( i );
		return dict;
	};
	void set_state( Model& model, const std::map< std::string, double >& dict ) {
		State s = model.GetState();
		for ( auto& [key, value] : dict )
			s.SetValue( key, value );
		model.SetState( s, 0.0 );
	};
	void log_measure_report( const Model& model ) {
		if ( model.GetMeasure() )
			log::info( model.GetMeasure()->GetReport() );
	}

	void check_array_length( size_t expected, size_t received ) {
		SCONE_ERROR_IF( expected != received, stringf( "Invalid array length; expected %d, received %d", expected, received ) );
	}

	void set_dof_positions( Model& model, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		auto& dofs = model.GetDofs();
		check_array_length( dofs.size(), v.shape( 0 ) );
		for ( index_t i = 0; i < dofs.size(); ++i )
			dofs[i]->SetPos( v( i ) );
	};

	void set_dof_velocities( Model& model, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		auto& dofs = model.GetDofs();
		check_array_length( dofs.size(), v.shape( 0 ) );
		for ( index_t i = 0; i < dofs.size(); ++i )
			dofs[i]->SetVel( v( i ) );
	};

	void init_state_from_dofs( Model& model ) {
		model.InitStateFromDofs();
	};

	void init_muscle_activations( Model& model, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		auto& mus = model.GetMuscles();
		check_array_length( mus.size(), v.shape( 0 ) );
		for ( index_t i = 0; i < mus.size(); ++i )
			mus[i]->InitializeActivation( v( i ) );
	};

	template< typename T, typename C, typename F > py::array_t<T> extract_array( const C& cont, F fn ) {
		auto [v, p] = make_array<T>( std::size( cont ) );
		for ( index_t i = 0; i < std::size( cont ); ++i )
			p[i] = static_cast<T>( fn( cont[i] ) );
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

	void set_actuator_inputs( Model& model, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		check_array_length( model.GetActuators().size(), v.shape( 0 ) );

		if ( auto* ec = dynamic_cast<ExternalController*>( model.GetController() ) ) {
			for ( index_t i = 0; i < ec->GetActuatorCount(); ++i )
				ec->SetInput( i, v( i ) );
		}
		else {
			auto& act = model.GetActuators();
			for ( index_t i = 0; i < act.size(); ++i ) {
				act[i]->ClearInput();
				act[i]->AddInput( v( i ) );
			}
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

	void set_delayed_actuator_inputs( Model& model, const py::array_t<double>& values ) {
		auto v = values.unchecked<1>();
		check_array_length( model.GetActuators().size(), v.shape( 0 ) );

		if ( auto* ec = dynamic_cast<ExternalController*>( model.GetController() ) ) {
			for ( index_t i = 0; i < ec->GetActuatorCount(); ++i )
				ec->SetDelayedInput( i, v( i ) );
		}
		else
		{
			auto& dag = model.GetDelayedActuatorGroup();
			if ( dag.actuators_.empty() )
				create_delayed_actuators( model );
			for ( index_t i = 0; i < dag.actuators_.size(); ++i ) {
				dag.actuators_[i].first->ClearInput();
				dag.actuators_[i].second.back() = v( i );
			}
		}
	};

	fs::path to_fs( const xo::path& p ) { return fs::path( p.str() ); }
	xo::path from_fs( const fs::path& p ) { return xo::path( p.string() ); }

	void write_results( const Model& m, std::string dir, std::string filename ) {
		SCONE_ASSERT( !m.GetExternalResources().empty() );
		auto target_dir = to_fs( GetFolder( SconeFolder::Results ) / dir );
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
