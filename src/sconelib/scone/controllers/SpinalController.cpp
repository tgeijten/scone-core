#include "SpinalController.h"

#include "scone/model/Model.h"
#include "snel/update.h"
#include "scone/model/Sensors.h"
#include "scone/model/MuscleId.h"
#include "scone/model/Muscle.h"
#include "scone/core/Log.h"
#include "scone/core/profiler_config.h"
#include "snel/snel_tools.h"

namespace scone
{
	using xo::uint32;

	SpinalController::SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		INIT_MEMBER_REQUIRED( pn, neural_delays_ ),
		INIT_MEMBER_REQUIRED( pn, activation_ )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		InitMuscleInfo( pn, model );

		// create delayed muscle sensors and actuators
		for ( auto& mus : model.GetMuscles() ) {
			auto nd = GetNeuralDelay( *mus );
			auto& sp = model.AcquireSensor<MuscleLengthSensor>( *mus );
			l_sensors_.push_back( model.GetDelayedSensor( sp, nd ) );
			actuators_.push_back( model.GetDelayedActuator( *mus, nd ) );
		}

		l_group_ = AddMuscleNeurons( "L", pn, par );

		auto* ves_pn = pn.try_get_child( "VES" );
		if ( ves_pn ) {
			ves_group_ = network_.add_group( snel::no_update );
			const auto& body = *FindByName( model.GetBodies(), ves_pn->get_str( "body" ) );
			const auto& dir = Vec3::unit_z();
			for ( auto side : { Side::Right, Side::Left } ) {
				auto& sensor = model.AcquireSensor<BodyOriVelSensor>( body, dir, 0.2, "_z", side, 0.0 );
				ves_sensors_.push_back( model.GetDelayedSensor( sensor, ves_pn->get<Real>( "delay" ) ) );
				AddNeuron( ves_group_, "BOV_z" + GetSideName( side ), 0.0 );
			}
		}

		// add neuron groups
		auto ia_group = AddGroupNeurons( "IA", pn, par );
		mn_group_ = AddMuscleNeurons( "MN", pn, par );

		// connect IA interneurons
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			auto& mg = muscle_groups_[ mgi ];
			for ( auto mi : mg.muscle_indices_ )
				Connect( l_group_, mi, ia_group, mgi, par, pn, "L_IA_weight", mg.muscle_indices_.size() );
			for ( auto amgi : mg.ant_group_indices_ )
				Connect( ia_group, amgi, ia_group, mgi, par, pn, "IA_IA_weight", mg.ant_group_indices_.size() );
			if ( ves_group_ )
				for ( uint32 vi = 0; vi < network_.group_size( ves_group_ ); ++vi )
					Connect( ves_group_, vi, ia_group, mgi, par, pn, "VES_IA_weight", 1 );
		}

		// connect motor units
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			Connect( l_group_, mi, mn_group_, mi, par, pn, "L_MN_weight", 1 );
			for ( auto amgi : muscles_[ mi ].ant_group_indices_ )
				Connect( ia_group, amgi, mn_group_, mi, par, pn, "IA_MN_weight", muscles_[ mi ].ant_group_indices_.size() );
		}
	}

	bool SpinalController::ComputeControls( Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		auto& muscles = model.GetMuscles();
		for ( uint32 mi = 0; mi < muscles.size(); ++mi )
			network_.set_value( l_group_, mi, snel::real( l_sensors_[ mi ].GetValue() ) );
		for ( uint32 vi = 0; vi < ves_sensors_.size(); ++vi ) {
			auto v = snel::real( ves_sensors_[ vi ].GetValue() );
			network_.set_value( ves_group_, vi, v );
		}

		network_.update();

		for ( uint32 mi = 0; mi < muscles.size(); ++mi )
			actuators_[ mi ].AddInput( network_.value( mn_group_, mi ) );

		return false;
	}

	void SpinalController::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const {
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		for ( index_t i = 0; i < network_.neuron_count(); ++i )
			frame[ "sn." + neuron_names_[ i ] ] = network_.values_[ i ];
	}

	PropNode SpinalController::GetInfo() const { return PropNode(); }
	String SpinalController::GetClassSignature() const { return "SN"; }

	void SpinalController::InitMuscleInfo( const PropNode& pn, Model& model )
	{
		// setup muscle info
		for ( auto& mus : model.GetMuscles() )
			muscles_.emplace_back( mus->GetName() );

		for ( auto& [key, mgpn] : pn.select( "MuscleGroup" ) ) {
			for ( auto side : { Side::Left, Side::Right } ) {
				auto& mg = muscle_groups_.emplace_back( mgpn, side );
				auto& muscle_pattern = mgpn.get<xo::pattern_matcher>( "muscles" );
				for ( uint32 mi = 0; mi < muscles_.size(); ++mi )
					if ( mg.side_ == muscles_[ mi ].side_ && muscle_pattern.match( muscles_[ mi ].name_ ) )
						mg.muscle_indices_.emplace_back( mi );
				if ( mg.muscle_indices_.empty() )
					muscle_groups_.pop_back();
			}
		}

		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			for ( auto mi : muscle_groups_[ mgi ].muscle_indices_ )
				muscles_[ mi ].group_indices_.push_back( mgi );
			auto antagonists = muscle_groups_[ mgi ].pn_.try_get<xo::pattern_matcher>( "antagonists" );
			for ( uint32 amgi = 0; amgi < muscle_groups_.size(); ++amgi ) {
				if ( muscle_groups_[ mgi ].side_ == muscle_groups_[ amgi ].side_ ) {
					if ( antagonists && antagonists->match( muscle_groups_[ amgi ].name_ ) ) {
						muscle_groups_[ mgi ].ant_group_indices_.emplace_back( amgi );
						for ( auto mi : muscle_groups_[ mgi ].muscle_indices_ )
							muscles_[ mi ].ant_group_indices_.push_back( amgi );
					}
				}
			}
		}
	}

	TimeInSeconds SpinalController::GetNeuralDelay( const Muscle& m ) const
	{
		auto it = neural_delays_.find( MuscleId( m.GetName() ).base_ );
		SCONE_ERROR_IF( it == neural_delays_.end(), "Could not find neural delay for " + m.GetName() );
		return it->second;
	}

	snel::neuron_id SpinalController::AddNeuron( snel::group_id group, String name, Real bias )
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		neuron_names_.emplace_back( std::move( name ) );
		return network_.add_neuron( group, snel::real( bias ) );
	}

	snel::neuron_id SpinalController::AddNeuron( snel::group_id group, String name, Params& par, const PropNode& pn, const string& pinf )
	{
		auto bias = !pinf.empty() ? par.try_get( GetNameNoSide( name ), pn, pinf, 0.0 ) : 0.0;
		return AddNeuron( group, name, bias );
	}

	snel::group_id SpinalController::AddMuscleNeurons( String name, const PropNode& pn, Params& par )
	{
		auto gid = network_.add_group( snel::get_update_fn( pn.get<string>( name + "_activation", activation_ ) ) );
		auto bias_inf = name + "_bias";
		for ( auto& mus : muscles_ )
			AddNeuron( gid, name + '.' + mus.name_, par, pn, bias_inf );
		return gid;
	}

	snel::group_id SpinalController::AddGroupNeurons( String name, const PropNode& pn, Params& par )
	{
		auto gid = network_.add_group( snel::get_update_fn( pn.get<string>( name + "_activation", activation_ ) ) );
		auto bias_inf = name + "_bias";
		for ( auto& mg : muscle_groups_ )
			AddNeuron( gid, name + '.' + mg.sided_name(), par, pn, bias_inf );
		return gid;
	}

	snel::group_id SpinalController::AddInterNeurons( String name, size_t count, const PropNode& pn, Params& par )
	{
		auto gid = network_.add_group( snel::get_update_fn( pn.get<string>( name + "_activation", activation_ ) ) );
		auto bias_inf = name + "_bias";
		for ( auto side : { Side::Right, Side::Left } )
			for ( size_t i = 0; i < count; ++i )
				AddNeuron( gid, GetSidedName( name + '.' + to_str( i ), side ), par, pn, bias_inf );
		return gid;
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, uint32 sidx, snel::group_id tgid, uint32 tidx, Real weight )
	{
		return network_.connect( sgid, sidx, tgid, tidx, snel::real( weight ) );
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, uint32 sidx, snel::group_id tgid, uint32 tidx, Params& par, const PropNode& pn, const string& pinf, size_t size )
	{
		SCONE_ASSERT( size > 0 );
		auto snid = network_.get_id( sgid, sidx );
		auto tnid = network_.get_id( tgid, tidx );
		auto par_name = GetNameNoSide( neuron_names_[ tnid.value() ] ) + "-" + GetNameNoSide( neuron_names_[ snid.value() ] );
		auto weight = par.get( par_name, pn[ pinf ] ) / Real( size );
		return Connect( sgid, sidx, tgid, tidx, weight );
	}
}
