#include "SpinalController.h"

#include "scone/model/Model.h"
#include "snel/update.h"
#include "scone/model/Sensors.h"
#include "scone/model/MuscleId.h"
#include "scone/model/Muscle.h"
#include "scone/core/Log.h"
#include "scone/core/profiler_config.h"
#include "snel/update_tools.h"

namespace scone
{
	using xo::uint32;
	constexpr auto both_sides = { Side::Right, Side::Left };
	static const char* axis_names[] = { "x", "y", "z" };

	SpinalController::SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		INIT_MEMBER_REQUIRED( pn, neural_delays_ ),
		INIT_MEMBER_REQUIRED( pn, activation_ ),
		INIT_MEMBER( pn, planar, model.GetDofs().size() < 14 ),
		INIT_MEMBER( pn, neuron_equilibration_steps, 20 )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		InitMuscleInfo( pn, model );

		// create L neurons
		l_group_ = AddInputNeuronGroup( "L" );
		l_bias_ = pn.get<Real>( "L_bias", 0.0 );
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			auto& mus = *model.GetMuscles()[ mi ];
			auto& sp = model.AcquireSensor<MuscleLengthSensor>( mus );
			l_sensors_.push_back( model.GetDelayedSensor( sp, muscles_[ mi ].delay_ ) );
			AddNeuron( l_group_, mus.GetName(), 0.0 );
		}

		// create F neurons
		f_group_ = AddInputNeuronGroup( "F" );
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			auto& mus = *model.GetMuscles()[ mi ];
			auto& sp = model.AcquireSensor<MuscleForceSensor>( mus );
			f_sensors_.push_back( model.GetDelayedSensor( sp, muscles_[ mi ].delay_ ) );
			AddNeuron( f_group_, mus.GetName(), 0.0 );
		}

		// create VES neurons
		if ( auto* ves_pn = pn.try_get_child( "VES" ) ) {
			ves_group_ = AddInputNeuronGroup( "VES" );
			const auto& body = *FindByName( model.GetBodies(), ves_pn->get_str( "body" ) );
			for ( int axis = planar ? 2 : 0; axis < 3; ++axis ) {
				for ( auto side : both_sides ) {
					auto& sensor = model.AcquireSensor<BodyOriVelSensor>( body, Vec3::axis( axis ), 0.2, axis_names[ axis ], side, 0.0 );
					ves_sensors_.push_back( model.GetDelayedSensor( sensor, ves_pn->get<Real>( "delay" ) ) );
					AddNeuron( ves_group_, axis_names[ axis ] + GetSideName( side ), 0.0 );
				}
			}
		}

		// create LD neurons
		if ( auto* ld_pn = pn.try_get_child( "LD" ) ) {
			load_group_ = AddInputNeuronGroup( "LD" );
			for ( auto side : both_sides ) {
				auto& leg = model.GetLeg( Location( side ) );
				auto& sensor = model.AcquireSensor<LegLoadSensor>( leg );
				load_sensors_.push_back( model.GetDelayedSensor( sensor, ld_pn->get<Real>( "delay" ) ) );
				AddNeuron( load_group_, GetSidedName( "LD", side ), 0.0 );
			}
		}

		// CPG neurons
		if ( auto* cpg_pn = pn.try_get_child( "CPG" ) ) {
			cpg_group_ = AddNeuronGroup( "CPG", pn );
			for ( auto side : both_sides ) {
				auto flex_idx = AddNeuron( cpg_group_, GetSidedName( "flex", side ), pn, par );
				auto flex_pat = cpg_pn->get<xo::pattern_matcher>( "flex_inputs" );
				auto ext_idx = AddNeuron( cpg_group_, GetSidedName( "ext", side ), pn, par );
				auto ext_pat = cpg_pn->get<xo::pattern_matcher>( "ext_inputs" );
				Connect( cpg_group_, ext_idx, cpg_group_, flex_idx, par, pn, nullptr, 1 );
				for ( uint32 mi = 0; mi < muscles_.size(); ++mi )
					if ( muscles_[ mi ].side_ == side ) {
						if ( flex_pat.match( GetNeuronName( l_group_, mi ) ) )
							Connect( l_group_, mi, cpg_group_, flex_idx, par, pn, nullptr, 1 );
						if ( flex_pat.match( GetNeuronName( f_group_, mi ) ) )
							Connect( f_group_, mi, cpg_group_, flex_idx, par, pn, nullptr, 1 );
					}
				Connect( cpg_group_, flex_idx, cpg_group_, ext_idx, par, pn, nullptr, 1 );
				for ( uint32 mi = 0; mi < muscles_.size(); ++mi )
					if ( muscles_[ mi ].side_ == side ) {
						if ( ext_pat.match( GetNeuronName( l_group_, mi ) ) )
							Connect( l_group_, mi, cpg_group_, ext_idx, par, pn, nullptr, 1 );
						if ( ext_pat.match( GetNeuronName( f_group_, mi ) ) )
							Connect( f_group_, mi, cpg_group_, ext_idx, par, pn, nullptr, 1 );
					}
			}
		}

		// IA interneurons
		ia_group_ = AddMuscleGroupNeurons( "IA", pn, par );

		if ( pn.has_key( "IB_bias" ) )
			ib_group_ = AddMuscleGroupNeurons( "IB", pn, par );

		// add motor neurons
		mn_group_ = AddNeuronGroup( "MN", pn );
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			actuators_.push_back( model.GetDelayedActuator( *model.GetMuscles()[ mi ], muscles_[ mi ].delay_ ) );
			AddNeuron( mn_group_, muscles_[ mi ].name_, pn, par );
		}

		// add Renshaw cells
		if ( pn.has_key( "RC_bias" ) )
			rc_group_ = AddMuscleGroupNeurons( "RC", pn, par );

		// connect muscle group interneurons
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			auto& mg = muscle_groups_[ mgi ];

			// IA interneurons
			if ( ia_group_ ) {
				Connect( l_group_, mg.muscle_indices_, ia_group_, mgi, par, pn, &mg.pn_ );
				Connect( ia_group_, mg.ant_group_indices_, ia_group_, mgi, par, pn, &mg.pn_ );
			}

			// VES -> IA
			if ( ves_group_ )
				for ( uint32 vi = 0; vi < network_.group_size( ves_group_ ); ++vi )
					if ( GetNeuronSide( ves_group_, vi ) == mg.side_ )
						Connect( ves_group_, vi, ia_group_, mgi, par, pn, &mg.pn_, 1 );
			// Load -> IA
			if ( load_group_ && pn.has_key( "LD_IA_weight" ) )
				for ( uint32 vi = 0; vi < network_.group_size( load_group_ ); ++vi )
					Connect( load_group_, vi, ia_group_, mgi, par, pn, &mg.pn_, 1 );
			// CPG -> IA
			if ( cpg_group_ )
				for ( uint32 ci = 0; ci < network_.group_size( cpg_group_ ); ++ci )
					if ( GetNeuronSide( cpg_group_, ci ) == mg.side_ )
						Connect( cpg_group_, ci, ia_group_, mgi, par, pn, &mg.pn_, 1 );
			// IB -> IA
			if ( pn.has_key( "IB_IA_weight" ) )
				Connect( ib_group_, mgi, ia_group_, mgi, par, pn, &mg.pn_, 1 );

			// IB interneurons
			if ( ib_group_ ) {
				Connect( f_group_, mg.muscle_indices_, ib_group_, mgi, par, pn, &mg.pn_ );
				if ( pn.has_key( "L_IB_weight" ) )
					Connect( l_group_, mg.muscle_indices_, ib_group_, mgi, par, pn, &mg.pn_ );
				if ( load_group_ && pn.has_key( "LD_IB_weight" ) )
					for ( uint32 vi = 0; vi < network_.group_size( load_group_ ); ++vi )
						Connect( load_group_, vi, ib_group_, mgi, par, pn, &mg.pn_, 1 );
				Connect( ib_group_, mg.ant_group_indices_, ib_group_, mgi, par, pn, &mg.pn_ );
				Connect( ib_group_, mg.related_group_indices_, ib_group_, mgi, par, pn, &mg.pn_ );
			}

			// MN -> RC
			if ( rc_group_ ) {
				Connect( mn_group_, mg.muscle_indices_, rc_group_, mgi, par, pn, &mg.pn_ );
			}
		}

		// connect motor units
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			const PropNode* mg_pn = muscles_[ mi ].group_indices_.empty() ? nullptr : &muscle_groups_[ muscles_[ mi ].group_indices_.front() ].pn_;

			// monosynaptic L connections
			if ( l_group_ )
				Connect( l_group_, mi, mn_group_, mi, par, pn, mg_pn, 1 );

			// connect IAIN to antagonists
			if ( ia_group_ )
				Connect( ia_group_, muscles_[ mi ].ant_group_indices_.container(), mn_group_, mi, par, pn, mg_pn );

			// connect IBIN to group members
			if ( ib_group_ )
				Connect( ib_group_, muscles_[ mi ].group_indices_.container(), mn_group_, mi, par, pn, mg_pn );

			// CPG -> MN
			if ( cpg_group_ )
				for ( uint32 ci = 0; ci < network_.group_size( cpg_group_ ); ++ci )
					if ( GetNeuronSide( cpg_group_, ci ) == muscles_[ mi ].side_ )
						Connect( cpg_group_, ci, mn_group_, mi, par, pn, nullptr, 1 );

			// RC -> MN
			if ( rc_group_ ) {
				Connect( rc_group_, muscles_[ mi ].group_indices_.container(), mn_group_, mi, par, pn, mg_pn );
			}
		}
	}

	bool SpinalController::ComputeControls( Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		auto& muscles = model.GetMuscles();
		for ( uint32 mi = 0; mi < muscles.size(); ++mi ) {
			network_.set_value( l_group_, mi, snel::real( l_sensors_[ mi ].GetValue() + l_bias_ ) );
			network_.set_value( f_group_, mi, snel::real( f_sensors_[ mi ].GetValue() ) );
		}
		for ( uint32 vi = 0; vi < ves_sensors_.size(); ++vi )
			network_.set_value( ves_group_, vi, snel::real( ves_sensors_[ vi ].GetValue() ) );
		for ( uint32 vi = 0; vi < load_sensors_.size(); ++vi )
			network_.set_value( load_group_, vi, snel::real( load_sensors_[ vi ].GetValue() ) );

		network_.update();

		if ( timestamp == 0.0 )
			for ( int i = 0; i < neuron_equilibration_steps; ++i )
				network_.update();

		for ( uint32 mi = 0; mi < muscles.size(); ++mi )
			actuators_[ mi ].AddInput( network_.value( mn_group_, mi ) );

		return false;
	}

	String SpinalController::GetClassSignature() const { return stringf( "SN%d", network_.neurons_.size() ); }

	uint32 SpinalController::AddNeuron( snel::group_id gid, const String& name, Real bias )
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		neuron_names_.emplace_back( neuron_group_names_[ gid.value() ] + '.' + name );
		auto nid = network_.add_neuron( gid, snel::real( bias ) );
		return nid.value() - network_.groups_[ gid.value() ].neuron_begin_.value();
	}

	uint32 SpinalController::AddNeuron( snel::group_id gid, const String& name, const PropNode& pn, Params& par )
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		neuron_names_.emplace_back( neuron_group_names_[ gid.value() ] + '.' + name );
		auto bias = par.try_get( GetNameNoSide( neuron_names_.back() ), pn, neuron_group_names_[ gid.value() ] + "_bias", 0.0 );
		auto nid = network_.add_neuron( gid, snel::real( bias ) );
		return nid.value() - network_.groups_[ gid.value() ].neuron_begin_.value();
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
		//log::debug( GetNeuronName( sgid, sidx ), " -> ", GetNeuronName( tgid, tidx ), " ", par_name, "=", weight );
		return network_.connect( sgid, sidx, tgid, tidx, snel::real( weight ) );
	}

	snel::link_id SpinalController::Connect( snel::group_id sgid, uint32 sidx, snel::group_id tgid, uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, size_t size )
	{
		SCONE_ASSERT( size > 0 );
		auto s = 1.0 / Real( size );
		auto par_info_name = GroupName( sgid ) + '_' + GroupName( tgid ) + "_weight";
		const PropNode* mg_pn = pn2 ? pn2->try_get_child( par_info_name ) : nullptr;
		auto pname = GetParName( GetNeuronName( sgid, sidx ), GetNeuronName( tgid, tidx ) );
		auto weight = s * par.get( pname, mg_pn ? *mg_pn : pn.get_child( par_info_name ) );
		return Connect( sgid, sidx, tgid, tidx, weight );
	}

	void SpinalController::Connect( snel::group_id sgid, const index_vec& sidxvec, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2 )
	{
		for ( auto sidx : sidxvec )
			Connect( sgid, sidx, tgid, tidx, par, pn, pn2, sidxvec.size() );
	}

	void SpinalController::InitMuscleInfo( const PropNode& pn, Model& model )
	{
		// setup muscle info
		for ( auto& mus : model.GetMuscles() )
			muscles_.emplace_back( mus->GetName(), GetNeuralDelay( *mus ) );

		for ( auto& [key, mgpn] : pn.select( "MuscleGroup" ) ) {
			for ( auto side : both_sides ) {
				auto& mg = muscle_groups_.emplace_back( mgpn, side );
				auto& muscle_pat = mgpn.get<xo::pattern_matcher>( "muscles" );
				for ( uint32 mi = 0; mi < muscles_.size(); ++mi )
					if ( mg.side_ == muscles_[ mi ].side_ && muscle_pat.match( muscles_[ mi ].name_ ) )
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
			auto rpat = mg.pn_.try_get<xo::pattern_matcher>( "related" );
			auto cl_apat = mg.pn_.try_get<xo::pattern_matcher>( "cl_antagonists" );
			for ( uint32 mgi_other = 0; mgi_other < muscle_groups_.size(); ++mgi_other ) {
				if ( mgi != mgi_other ) {
					auto& mg_other = muscle_groups_[ mgi_other ];
					bool same_side = mg.side_ == mg_other.side_;
					bool is_antagonist = ( same_side && apat && apat->match( mg_other.name_ ) ) || ( !same_side && cl_apat && cl_apat->match( mg_other.name_ ) );
					bool is_related = same_side && rpat && rpat->match( mg_other.name_ );

					if ( is_antagonist ) {
						mg.ant_group_indices_.emplace_back( mgi_other );
						for ( auto mi : mg.muscle_indices_ )
							muscles_[ mi ].ant_group_indices_.insert( mgi_other );
					}
					else if ( is_related )
						mg.related_group_indices_.emplace_back( mgi_other );
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

	void SpinalController::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		for ( index_t i = 0; i < network_.neuron_count(); ++i ) {
			frame[ neuron_names_[ i ] ] = network_.values_[ i ];
			const auto& n = network_.neurons_[ i ];
			for ( auto lid = n.input_begin_; lid != n.input_end_; ++lid ) {
				const auto& l = network_.links_[ lid.value() ];
				auto v = network_.value( l.input_ ) * l.weight_;
				frame[ neuron_names_[ i ] + '-' + neuron_names_[ l.input_.value() ] ] = v;
			}
		}
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
		for ( auto& mg : muscle_groups_ ) {
			auto& mgpn = mgspn.add_child( mg.sided_name() );
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
}
