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
	constexpr auto both_sides = { Side::Right, Side::Left };

	SpinalController::SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		INIT_MEMBER_REQUIRED( pn, neural_delays_ ),
		INIT_MEMBER_REQUIRED( pn, activation_ ),
		INIT_MEMBER( pn, planar, model.GetDofs().size() < 14 )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		InitMuscleInfo( pn, model );

		// create L neurons
		l_group_ = AddInputNeuronGroup( "L" );
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			auto& mus = *model.GetMuscles()[ mi ];
			auto& sp = model.AcquireSensor<MuscleLengthSensor>( mus );
			l_sensors_.push_back( model.GetDelayedSensor( sp, muscles_[ mi ].delay_ ) );
			AddNeuron( l_group_, mus.GetName(), 0.0 );
		}

		// create VES neurons
		static const char* axis_name[] = { "x", "y", "z" };
		if ( auto* ves_pn = pn.try_get_child( "VES" ) ) {
			ves_group_ = AddInputNeuronGroup( "VES" );
			const auto& body = *FindByName( model.GetBodies(), ves_pn->get_str( "body" ) );
			for ( int axis = planar ? 2 : 0; axis < 3; ++axis ) {
				for ( auto side : both_sides ) {
					auto& sensor = model.AcquireSensor<BodyOriVelSensor>( body, Vec3::axis( axis ), 0.2, axis_name[ axis ], side, 0.0 );
					ves_sensors_.push_back( model.GetDelayedSensor( sensor, ves_pn->get<Real>( "delay" ) ) );
					AddNeuron( ves_group_, axis_name[ axis ] + GetSideName( side ), 0.0 );
				}
			}
		}

		// add neuron groups
		auto ia_group = AddMuscleGroupNeurons( "IA", pn, par );

		// add motor neurons
		mn_group_ = AddNeuronGroup( "MN", pn );
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			actuators_.push_back( model.GetDelayedActuator( *model.GetMuscles()[ mi ], muscles_[ mi ].delay_ ) );
			AddNeuron( mn_group_, muscles_[ mi ].name_, pn, par );
		}

		// connect IA interneurons
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			auto& mg = muscle_groups_[ mgi ];
			for ( auto mi : mg.muscle_indices_ )
				Connect( l_group_, mi, ia_group, mgi, par, pn, &mg, mg.muscle_indices_.size() );
			for ( auto amgi : mg.ant_group_indices_ )
				Connect( ia_group, amgi, ia_group, mgi, par, pn, &mg, mg.ant_group_indices_.size() );
			if ( ves_group_ )
				for ( uint32 vi = 0; vi < network_.group_size( ves_group_ ); ++vi )
					if ( mgi % 2 == vi % 2 ) // this is to match sides, might consider something nicer
						Connect( ves_group_, vi, ia_group, mgi, par, pn, &mg, 1 );
		}

		// connect motor units
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			MuscleGroup* mg = muscles_[ mi ].group_indices_.empty() ? nullptr : &muscle_groups_[ muscles_[ mi ].group_indices_.front() ];
			Connect( l_group_, mi, mn_group_, mi, par, pn, mg, 1 );
			for ( auto amgi : muscles_[ mi ].ant_group_indices_ )
				Connect( ia_group, amgi, mn_group_, mi, par, pn, mg, muscles_[ mi ].ant_group_indices_.size() );
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

	void SpinalController::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		for ( index_t i = 0; i < network_.neuron_count(); ++i )
			frame[ "sn." + neuron_names_[ i ] ] = network_.values_[ i ];
	}

	PropNode SpinalController::GetInfo() const {
		PropNode pn;
		auto& muscles_pn = pn.add_child( "Muscles" );
		for ( auto& m : muscles_ ) {
			auto& mpn = muscles_pn.add_child( m.name_ );
			mpn[ "delay" ] = m.delay_;
			mpn[ "groups" ] = m.group_indices_;
			mpn[ "antagonists" ] = m.ant_group_indices_;
		}
		auto& mgspn = pn.add_child( "MuscleGroups" );
		for ( auto& mg : muscle_groups_ ){
			auto& mgpn = mgspn .add_child( mg.sided_name() );
			mgpn[ "muscles" ] = mg.muscle_indices_;
			mgpn[ "antagonists" ] = mg.ant_group_indices_;
		}
		auto& nspn = pn.add_child( "Neurons" );
		for ( auto gid = snel::group_id( 0 ); gid.value() < network_.groups_.size(); ++( gid.value() ) ) {
			auto& gpn = nspn.add_child( neuron_group_names_[ gid.value() ] );
			for ( uint32 nidx = 0; nidx < network_.group_size( snel::group_id( gid ) ); ++nidx ) {
				auto nid = network_.get_id( gid, nidx );
				auto& n = network_.neurons_[ nid.value() ];
				auto& npn = gpn.add_child( neuron_names_[ nid.value() ] );
				npn[ "bias" ] = n.bias_;
				for ( auto lit = n.input_begin_.iter( network_.links_ ); lit != n.input_end_.iter( network_.links_ ); ++lit )
					npn[ neuron_names_[ lit->input_.value() ] ] = lit->weight_;
			}
		}
		return pn;
	}

	String SpinalController::GetClassSignature() const {
		auto s = stringf( "SN%d", network_.neurons_.size() );
		return s;
	}

	snel::neuron_id SpinalController::AddNeuron( snel::group_id gid, const String& name, Real bias )
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		neuron_names_.emplace_back( neuron_group_names_[ gid.value() ] + '.' + name );
		return network_.add_neuron( gid, snel::real( bias ) );
	}

	snel::neuron_id SpinalController::AddNeuron( snel::group_id gid, const String& name, const PropNode& pn, Params& par )
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		neuron_names_.emplace_back( neuron_group_names_[ gid.value() ] + '.' + name );
		auto bias = par.try_get( GetNameNoSide( neuron_names_.back() ), pn, neuron_group_names_[ gid.value() ] + "_bias", 0.0 );
		return network_.add_neuron( gid, snel::real( bias ) );
	}

	snel::group_id SpinalController::AddNeuronGroup( const String& name, const PropNode& pn )
	{
		neuron_group_names_.emplace_back( name );
		return network_.add_group( snel::get_update_fn( pn.get<string>( name + "_activation", activation_ ) ) );
	}

	snel::group_id SpinalController::AddInputNeuronGroup( const String& name )
	{
		neuron_group_names_.emplace_back( name );
		return network_.add_group( snel::no_update );
	}

	snel::group_id SpinalController::AddMuscleGroupNeurons( String name, const PropNode& pn, Params& par )
	{
		auto gid = AddNeuronGroup( name, pn );
		for ( auto& mg : muscle_groups_ )
			AddNeuron( gid, mg.sided_name(), pn, par );
		return gid;
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, uint32 sidx, snel::group_id tgid, uint32 tidx, Real weight )
	{
		return network_.connect( sgid, sidx, tgid, tidx, snel::real( weight ) );
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, uint32 sidx, snel::group_id tgid, uint32 tidx, Params& par, const PropNode& pn, const MuscleGroup* mg, size_t size )
	{
		SCONE_ASSERT( size > 0 );
		auto s = 1.0 / Real( size );
		auto par_info_name = GroupName( sgid ) + '_' + GroupName( tgid ) + "_weight";
		const PropNode* mg_pn = mg ? mg->pn_.try_get_child( par_info_name ) : nullptr;
		auto pinf = ParInfo( "", mg_pn ? *mg_pn : pn.get_child( par_info_name ), par.options() );
		auto pname = GetParName( GetNeuronName( sgid, sidx ), GetNeuronName( tgid, tidx ) );
		auto weight = pinf.is_constant() ? s * pinf.mean : par.get( pname, s * pinf.mean, s * pinf.std, s * pinf.min, s * pinf.max );
		//log::debug( GetNeuronName( sgid, sidx ), " -> ", GetNeuronName( tgid, tidx ), " ", par_name, "=", weight );
		return Connect( sgid, sidx, tgid, tidx, weight );
	}

	void SpinalController::InitMuscleInfo( const PropNode& pn, Model& model )
	{
		// setup muscle info
		for ( auto& mus : model.GetMuscles() )
			muscles_.emplace_back( mus->GetName(), GetNeuralDelay( *mus ) );

		for ( auto& [key, mgpn] : pn.select( "MuscleGroup" ) ) {
			for ( auto side : both_sides ) {
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
			auto& mg = muscle_groups_[ mgi ];
			for ( auto mi : mg.muscle_indices_ )
				muscles_[ mi ].group_indices_.insert( mgi );
			auto apat = mg.pn_.try_get<xo::pattern_matcher>( "antagonists" );
			auto cl_apat = mg.pn_.try_get<xo::pattern_matcher>( "cl_antagonists" );
			for ( uint32 amgi = 0; amgi < muscle_groups_.size(); ++amgi ) {
				auto& amg = muscle_groups_[ amgi ];
				if ( ( apat && mg.side_ == amg.side_ && apat->match( amg.name_ ) ) ||
					( cl_apat && mg.side_ != amg.side_ && cl_apat->match( amg.name_ ) ) )
				{
					mg.ant_group_indices_.emplace_back( amgi );
					for ( auto mi : mg.muscle_indices_ )
						muscles_[ mi ].ant_group_indices_.insert( amgi );
				}
			}
		}

		for ( auto& m : muscles_ )
			if ( m.group_indices_.empty() )
				log::warning( m.name_, " is not part of any MuscleGroup" );
	}

	TimeInSeconds SpinalController::GetNeuralDelay( const Muscle& m ) const
	{
		auto it = neural_delays_.find( MuscleId( m.GetName() ).base_ );
		SCONE_ERROR_IF( it == neural_delays_.end(), "Could not find neural delay for " + m.GetName() );
		return it->second;
	}

	const string SpinalController::GetParName( const string& src, const string& trg ) const
	{
		if ( GetSideFromName( src ) == GetSideFromName( trg ) )
			return GetNameNoSide( trg ) + "-" + GetNameNoSide( src );
		else return GetNameNoSide( trg ) + "-" + GetNameNoSide( src ) + "_o";
	}
}
