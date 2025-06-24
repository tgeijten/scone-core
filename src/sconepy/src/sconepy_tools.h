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
#include "scone/controllers/CompositeController.h"
#include "scone/optimization/Optimizer.h"
#include "spot/static_par_set.h"

namespace fs = std::filesystem;

namespace scone
{
	void evaluate_par_file( const std::string& file ) {
		auto scenario_pn = LoadScenario( file );
		log::info( "Evaluating ", file );
		auto results = EvaluateScenario( scenario_pn, file, file );
		log::info( results );
	}

	ExternalController* TryGetExternalController( Model& model ) {
		auto* c = model.GetController();
		if ( auto* ec = dynamic_cast<ExternalController*>( c ) )
			return ec;
		else if ( auto* cc = dynamic_cast<CompositeController*>( c ) )
			if ( auto* ec = cc->TryGetChild<ExternalController>() )
				return ec;
		return nullptr;
	}

	bool CreateExternalController( Model& model ) {
		PropNode cpn{ { "ExternalController", PropNode() } };
		auto fp = FindFactoryProps( GetControllerFactory(), cpn, "Controller" );
		spot::null_objective_info par;
		if ( auto* con = model.GetController() )  {
			if ( auto* cc = dynamic_cast<CompositeController*>( con ) )
				cc->InsertChildController( CreateController( fp, par, model, Location() ) );
			else {
				log::warning( "Cannot create ExternalController because Model Controller is no CompositeController" );
					return false;
			}
		}
		else {
			model.CreateController( fp, par );
		}
		return true;
	}

	ModelUP load_model( const std::string& file, const std::string& par_file = "" ) {
		auto file_path = xo::path( file );
		auto model_pn = xo::load_zml( file_path );
		auto model_factory_props_ = FindFactoryProps( GetModelFactory(), model_pn, "Model" );
		spot::static_par_set par;
		if ( !par_file.empty() )
			par.load( par_file );
		auto model = CreateModel( model_factory_props_, par, file_path.parent_path() );
		model->GetUserData().add_child( g_scenario_user_data_key, model_pn ); // add model_pn so we can save config.scone to results folder
		CreateExternalController( *model );
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

	void set_optimizer_console_output( Optimizer& opt ) {
		opt.SetOutputMode( Optimizer::console_output );
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

		// also init ExternalController values, because InitializeController() may be called via InitStateFromDofs() later
		if ( auto* ec = TryGetExternalController( model ) ) {
			for ( index_t i = 0; i < mus.size(); ++i )
				ec->SetInput( i, v( i ) );
		}
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

		if ( auto* ec = TryGetExternalController( model ) ) {
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

	int set_control_parameter( Model& model, const String& name, double value ) {
		if ( auto* c = model.GetController() )
			return c->TrySetControlParameter( name, value );
		else return 0;
	};

	double get_control_parameter( Model& model, const String& name ) {
		if ( auto* c = model.GetController() )
			if ( auto v = c->TryGetControlParameter( name ) )
				return *v;
		SCONE_ERROR( "Could not find control parameter " + name );
	};

	std::vector<String> get_control_parameter_names( Model& model ) {
		return model.GetController()->GetControlParameters();
	};

	double get_ray_distance( Model& model, const Vec3& pos, const Vec3& dir, double max_dist ) {
		return model.GetRayIntersection( pos, dir, max_dist ).first;
	}

	fs::path to_fs( const xo::path& p ) { return fs::path( p.str() ); }
	xo::path from_fs( const fs::path& p ) { return xo::path( p.string() ); }

	void write_results( const Model& m, std::string dir, std::string filename ) {
		SCONE_ASSERT( !m.GetExternalResources().IsEmpty() );
		auto xo_target_dir = GetFolder( SconeFolder::Results ) / dir;
		auto target_dir = to_fs( xo_target_dir );
		if ( !fs::exists( target_dir ) )
		{
			// setup output folder
			fs::create_directories( target_dir );
			xo::save_file( m.GetUserData().get_child( g_scenario_user_data_key ), from_fs( target_dir ) / "config.scone" );
			m.GetExternalResources().WriteTo( xo_target_dir );
		}
		m.WriteResults( xo_target_dir / filename );
	};
}
