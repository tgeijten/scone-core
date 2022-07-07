#pragma once

#include <string>
#include <filesystem>
#include "xo/filesystem/path.h"
#include "scone/optimization/opt_tools.h"
#include "scone/model/Actuator.h"
#include "xo/serialization/prop_node_serializer_zml.h"
#include "scone/core/Factories.h"
#include "spot/par_io.h"
#include "scone/model/Muscle.h"
#include "scone/model/Dof.h"

namespace fs = std::filesystem;

namespace scone
{
	void evaluate_par_file( const std::string& file ) {
		xo::path scenario_file = scone::FindScenario( file );
		auto scenario_pn = scone::LoadScenario( scenario_file, true ); // for compatibility of versions < 2.0.0
		auto out_path = scenario_file.parent_path();
		log::info( "Evaluating ", file );
		auto results = scone::EvaluateScenario( scenario_pn, file, file );
		log::info( results );
	}
	scone::ModelUP load_model( const std::string& file ) {
		auto file_path = xo::path( file );
		auto model_pn = xo::load_zml( file_path );
		auto model_props = FindFactoryProps( GetModelFactory(), model_pn, "Model" );
		spot::null_objective_info par;
		auto model = scone::CreateModel( model_props, par, file_path.parent_path() );
		model->AddExternalResource( file_path ); // add .scone file so that it can be copied to results
		return model;
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

	void check_array_length( size_t expected, size_t received ) {
		SCONE_ERROR_IF( expected != received, stringf( "Invalid array length; expected %d, received %d", expected, received ) );
	}

	void set_actuator_values( scone::Model& m, const std::vector<double>& values ) {
		check_array_length( m.GetActuators().size(), values.size() );
		auto value_it = values.begin();
		for ( auto& act : m.GetActuators() ) {
			act->ClearInput();
			act->AddInput( *value_it++ );
		}
	};

	void set_dof_values( scone::Model& m, const std::vector<double>& values ) {
		check_array_length( m.GetDofs().size() * 2, values.size() );
		auto value_it = values.begin();
		for ( auto& dof : m.GetDofs() )
			dof->SetPos( *value_it++ );
		for ( auto& dof : m.GetDofs() )
			dof->SetVel( *value_it++ );
		m.UpdateStateFromDofs();
	};

	void init_muscle_activations( scone::Model& m, const std::vector<double>& values ) {
		check_array_length( m.GetMuscles().size(), values.size() );
		auto value_it = values.begin();
		for ( auto& act : m.GetMuscles() ) 
			act->InitializeActivation( *value_it++ );
	};

	template< typename C, typename F > std::vector<double> extract_vec( const C& cont, F fn ) {
		std::vector<double> v;
		v.reserve( std::size( cont ) );
		for ( auto& e : cont )
			v.emplace_back( fn( e ) );
		return v;
	};

	std::vector<double> get_muscle_lengths( const scone::Model& model ) {
		return extract_vec( model.GetMuscles(), []( const Muscle* m ) { return m->GetNormalizedFiberLength(); } );
	};
	std::vector<double> get_muscle_velocities( const scone::Model& model ) {
		return extract_vec( model.GetMuscles(), []( const Muscle* m ) { return m->GetNormalizedFiberVelocity(); } );
	};
	std::vector<double> get_muscle_forces( const scone::Model& model ) {
		return extract_vec( model.GetMuscles(), []( const Muscle* m ) { return m->GetNormalizedForce(); } );
	};
	std::vector<double> get_muscle_activations( const scone::Model& model ) {
		return extract_vec( model.GetMuscles(), []( const Muscle* m ) { return m->GetActivation(); } );
	};
	std::vector<double> get_muscle_excitations( const scone::Model& model ) {
		return extract_vec( model.GetMuscles(), []( const Muscle* m ) { return m->GetExcitation(); } );
	};
	std::vector<double> get_actuator_values( const scone::Model& model ) {
		return extract_vec( model.GetActuators(), []( const Actuator* m ) { return m->GetInput(); } );
	};
	std::vector<double> get_dof_values( const scone::Model& model ) {
		auto dof_pos = extract_vec( model.GetDofs(), []( const Dof* d ) { return d->GetPos(); } );
		return xo::append( dof_pos, extract_vec( model.GetDofs(), []( const Dof* d ) { return d->GetVel(); } ) );
	};

	fs::path to_fs( const xo::path& p ) { return fs::path( p.str() ); }

	void write_results( const scone::Model& m, std::string f ) {
		SCONE_ASSERT( !m.GetExternalResources().empty() );
		scone::ReplaceStringTags( f );
		auto target_dir = scone::GetFolder( scone::SCONE_RESULTS_FOLDER ) / f;
		fs::create_directories( to_fs( target_dir ) );
		for ( auto& p : m.GetExternalResources() )
			fs::copy( to_fs( p ), to_fs( target_dir ), fs::copy_options::overwrite_existing );
		m.WriteResults( target_dir / m.GetExternalResources().back().stem() );
	};
}
