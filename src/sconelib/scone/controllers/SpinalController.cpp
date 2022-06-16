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
	using snel::group_id, snel::link_id;
	constexpr auto both_sides = { Side::Right, Side::Left };
	static const char* axis_names[] = { "x", "y", "z" };
	std::vector<Side> get_sides( const Location& loc ) { if ( loc.GetSide() == Side::None ) return both_sides; else return { loc.GetSide() }; }

	SpinalController::SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc ),
		INIT_MEMBER( pn, neural_delays_, {} ),
		INIT_MEMBER_REQUIRED( pn, activation_ ),
		INIT_MEMBER( pn, planar, model.GetDofs().size() < 14 ),
		INIT_MEMBER( pn, neuron_equilibration_steps, 20 )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		auto sides = get_sides( loc );
		InitMuscleInfo( pn, model, loc );

		// create L neurons
		l_group_ = AddInputNeuronGroup( "L" );
		l_bias_ = pn.get<Real>( "L_bias", 0.0 );
		for ( auto& musinf : muscles_ ) {
			auto& mus = *model.GetMuscles()[ musinf.index_ ];
			auto& sp = model.AcquireSensor<MuscleLengthSensor>( mus );
			l_sensors_.push_back( model.GetDelayedSensor( sp, musinf.delay_ ) );
			AddNeuron( l_group_, mus.GetName(), 0.0 );
		}

		// create F neurons
		f_group_ = AddInputNeuronGroup( "F" );
		for ( auto& musinf : muscles_ ) {
			auto& mus = *model.GetMuscles()[ musinf.index_ ];
			auto& sp = model.AcquireSensor<MuscleForceSensor>( mus );
			f_sensors_.push_back( model.GetDelayedSensor( sp, musinf.delay_ ) );
			AddNeuron( f_group_, mus.GetName(), 0.0 );
		}

		// create VES neurons
		if ( auto* ves_pn = pn.try_get_child( "VES" ) ) {
			ves_group_ = AddInputNeuronGroup( "VES" );
			const auto& body = *FindByName( model.GetBodies(), ves_pn->get_str( "body" ) );
			ves_vel_gain_ = ves_pn->get<Real>( "vel_gain", 0.2 );
			auto ves_delay = ves_pn->get<Real>( "delay" );
			for ( int axis = planar ? 2 : 0; axis < 3; ++axis ) {
				for ( auto side : sides ) {
					auto& bp = model.AcquireSensor<BodyOrientationSensor>( body, Vec3::axis( axis ), axis_names[ axis ], side );
					ves_sensors_.push_back( model.GetDelayedSensor( bp, ves_delay ) );
					AddNeuron( ves_group_, string( "p" ) + axis_names[ axis ] + GetSideName( side ), 0.0 );
					if ( ves_vel_gain_ != 0.0 )	{
						auto& bv = model.AcquireSensor<BodyAngularVelocitySensor>( body, Vec3::axis( axis ), axis_names[ axis ], side, ves_vel_gain_ );
						ves_sensors_.push_back( model.GetDelayedSensor( bv, ves_delay ) );
						AddNeuron( ves_group_, string( "v" ) + axis_names[ axis ] + GetSideName( side ), 0.0 );
					}
				}
			}
		}

		// create LD neurons
		if ( auto* ld_pn = pn.try_get_child( "LD" ) ) {
			load_group_ = AddInputNeuronGroup( "LD" );
			for ( auto side : sides ) {
				auto& leg = model.GetLeg( Location( side ) );
				auto gain = ld_pn->get<Real>( "gain", 1.0 );
				auto offset = ld_pn->get<Real>( "bias", 0.0 );
				auto range = ld_pn->get<xo::boundsd>( "bounds", xo::boundsd::infinite() );
				auto& sensor = model.AcquireSensor<LegLoadSensor>( leg, gain, offset, range );
				load_sensors_.push_back( model.GetDelayedSensor( sensor, ld_pn->get<Real>( "delay" ) ) );
				AddNeuron( load_group_, GetSidedName( stringf( "leg%d", leg.GetRank() ), leg.GetSide() ), 0.0 );
			}
		}

		// IA interneurons
		ia_group_ = AddMuscleGroupNeurons( "IA", pn, par );

		// IB interneurons
		if ( pn.has_key( "IB_bias" ) )
			ib_group_ = AddMuscleGroupNeurons( "IB", pn, par );
		if ( pn.has_key( "IBI_bias" ) )
			ibi_group_ = AddMuscleGroupNeurons( "IBI", pn, par );
		if ( pn.has_key( "IBE_bias" ) )
			ibe_group_ = AddMuscleGroupNeurons( "IBE", pn, par );

		// PM neurons (premotor neurons)
		if ( pn.has_key( "PM_bias" ) )
			pm_group_ = AddMuscleGroupNeurons( "PM", pn, par );

		// MN neurons (motor neurons)
		mn_group_ = AddNeuronGroup( "MN", pn );
		for ( auto& musinf : muscles_ ) {
			auto* mgpn = !musinf.group_indices_.empty() ? &muscle_groups_[ musinf.group_indices_.front() ].pn_ : nullptr;
			actuators_.push_back( model.GetDelayedActuator( *model.GetMuscles()[ musinf.index_ ], musinf.delay_ ) );
			AddNeuron( mn_group_, musinf.name_, par, pn, mgpn );
		}

		// RC neurons (Renshaw cells)
		if ( pn.has_key( "RC_bias" ) ) {
			rc_group_ = AddNeuronGroup( "RC", pn );
			for ( auto& musinf : muscles_ )
				AddNeuron( rc_group_, musinf.name_, par, pn );
		}

		// connect muscle group interneurons
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			auto& mg = muscle_groups_[ mgi ];
			auto* contra_mg = mg.contra_group_index_ < muscle_groups_.size() ? &muscle_groups_[ mg.contra_group_index_ ] : nullptr;

			// IA interneurons
			if ( ia_group_ ) {
				Connect( l_group_, mg.muscle_indices_, ia_group_, mgi, par, pn, &mg.pn_ );
				Connect( ia_group_, mg.ant_group_indices_, ia_group_, mgi, par, pn, &mg.pn_ );
			}
			// VES -> IA
			if ( ves_group_ )
				for ( uint32 vi = 0; vi < network_.group_size( ves_group_ ); ++vi )
					if ( NeuronSide( ves_group_, vi ) == mg.side_ )
						TryConnect( ves_group_, vi, ia_group_, mgi, par, pn, &mg.pn_ );
			// Load -> IA
			if ( load_group_ )
				for ( uint32 vi = 0; vi < network_.group_size( load_group_ ); ++vi )
					TryConnect( load_group_, vi, ia_group_, mgi, par, pn, &mg.pn_,
						NeuronSide( load_group_, vi ) == mg.side_ ? "_weight" : "_com_weight" );

			// IB -> IA
			if ( pn.has_key( "IB_IA_weight" ) )
				Connect( ib_group_, mgi, ia_group_, mgi, par, pn, &mg.pn_ );
			if ( pn.has_key( "IBE_IA_weight" ) )
				Connect( ibe_group_, mgi, ia_group_, mgi, par, pn, &mg.pn_ );
			// RC -> IA
			if ( rc_group_ )
				Connect( rc_group_, mg.muscle_indices_, ia_group_, mgi, par, pn, &mg.pn_ );
			// PM -> IA
			if ( pm_group_ )
				Connect( pm_group_, mgi, ia_group_, mgi, par, pn, &mg.pn_ );

			// IB interneurons
			for ( auto ib_group : { ib_group_, ibi_group_, ibe_group_ } ) {
				if ( ib_group ) {
					auto group_suffix = "_" + GroupName( ib_group ) + "_weight";
					// F+L -> IB
					Connect( f_group_, mg.muscle_indices_, ib_group, mgi, par, pn, &mg.pn_ );
					TryConnect( l_group_, mg.muscle_indices_, ib_group, mgi, par, pn, &mg.pn_ );
					// LD -> IB
					if ( load_group_ )
						for ( uint32 vi = 0; vi < network_.group_size( load_group_ ); ++vi )
							TryConnect( load_group_, vi, ib_group, mgi, par, pn, &mg.pn_,
								NeuronSide( load_group_, vi ) == mg.side_ ? "_weight" : "_com_weight" );
					// VES -> IB
					if ( ves_group_ )
						for ( uint32 vi = 0; vi < network_.group_size( ves_group_ ); ++vi )
							if ( NeuronSide( ves_group_, vi ) == mg.side_ )
								TryConnect( ves_group_, vi, ib_group, mgi, par, pn, &mg.pn_ );
					// IB -> IB / IBE -> IBE / IBI -> IBI
					TryConnect( ib_group, mg.ant_group_indices_, ib_group, mgi, par, pn, &mg.pn_, "_ant_weight" );
					TryConnect( ib_group, mg.related_group_indices_, ib_group, mgi, par, pn, &mg.pn_, "_rel_weight" );
					if ( contra_mg ) {
						TryConnect( ib_group, mg.contra_group_index_, ib_group, mgi, par, pn, &mg.pn_, "_com_weight" );
						TryConnect( ib_group, contra_mg->ant_group_indices_, ib_group, mgi, par, pn, &mg.pn_, "_com_ant_weight" );
					}

					// IBE -> IBI / IBI -> IBE
					if ( ib_group == ibi_group_ || ib_group == ibe_group_ && ibi_group_ && ibe_group_ ) {
						auto src_group = ( ib_group == ibi_group_ ) ? ibe_group_ : ibi_group_;
						TryConnect( src_group, mg.ant_group_indices_, ib_group, mgi, par, pn, &mg.pn_, "_ant_weight" );
						TryConnect( src_group, mg.related_group_indices_, ib_group, mgi, par, pn, &mg.pn_, "_rel_weight" );
						if ( contra_mg ) {
							TryConnect( src_group, mg.contra_group_index_, ib_group, mgi, par, pn, &mg.pn_, "_com_weight" );
							TryConnect( src_group, contra_mg->ant_group_indices_, ib_group, mgi, par, pn, &mg.pn_, "_com_ant_weight" );
						}
					}
				}
			}

			if ( pm_group_ ) {
				// IB -> PM
				for ( uint32 smgi = 0; smgi < muscle_groups_.size(); ++smgi ) {
					if ( mg.side_ == muscle_groups_[ smgi ].side_ ) {
						if ( ib_group_ )
							Connect( ib_group_, smgi, pm_group_, mgi, par, pn, &mg.pn_ );
						if ( ibi_group_ )
							Connect( ibi_group_, smgi, pm_group_, mgi, par, pn, &mg.pn_ );
						if ( ibe_group_ )
							Connect( ibe_group_, smgi, pm_group_, mgi, par, pn, &mg.pn_ );
					}
				}
				// VES -> PM
				if ( ves_group_ && pn.has_key( "VES_PM_weight" ) )
					for ( uint32 vi = 0; vi < network_.group_size( ves_group_ ); ++vi )
						if ( NeuronSide( ves_group_, vi ) == mg.side_ )
							TryConnect( ves_group_, vi, pm_group_, mgi, par, pn, &mg.pn_ );
			}
		}

		// connect motor units
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			const PropNode* mg_pn = muscles_[ mi ].group_indices_.empty() ? nullptr : &muscle_groups_[ muscles_[ mi ].group_indices_.front() ].pn_;

			// monosynaptic L connections
			if ( l_group_ )
				Connect( l_group_, mi, mn_group_, mi, par, pn, mg_pn );

			// connect IAIN to antagonists
			if ( ia_group_ )
				Connect( ia_group_, muscles_[ mi ].ant_group_indices_.container(), mn_group_, mi, par, pn, mg_pn );

			// connect IBIN to group members
			if ( ib_group_ )
				Connect( ib_group_, muscles_[ mi ].group_indices_.container(), mn_group_, mi, par, pn, mg_pn );
			if ( ibi_group_ )
				Connect( ibi_group_, muscles_[ mi ].group_indices_.container(), mn_group_, mi, par, pn, mg_pn );
			if ( ibe_group_ )
				Connect( ibe_group_, muscles_[ mi ].group_indices_.container(), mn_group_, mi, par, pn, mg_pn );

			// PM -> MN
			if ( pm_group_ )
				Connect( pm_group_, muscles_[ mi ].group_indices_.container(), mn_group_, mi, par, pn, mg_pn );

			// RC -> MN
			if ( rc_group_ ) {
				Connect( rc_group_, mi, mn_group_, mi, par, pn, mg_pn );
				Connect( mn_group_, mi, rc_group_, mi, par, pn, mg_pn );
			}
		}
	}

	bool SpinalController::ComputeControls( Model& model, double timestamp )
	{
		SCONE_PROFILE_FUNCTION( model.GetProfiler() );

		// update L and F
		for ( uint32 mi = 0; mi < muscles_.size(); ++mi ) {
			network_.set_value( l_group_, mi, snel::real( l_sensors_[ mi ].GetValue() + l_bias_ ) );
			network_.set_value( f_group_, mi, snel::real( f_sensors_[ mi ].GetValue() ) );
		}
		// update VES
		for ( uint32 vi = 0; vi < ves_sensors_.size(); ++vi )
			network_.set_value( ves_group_, vi, snel::real( ves_sensors_[ vi ].GetValue() ) );
		// update LD
		for ( uint32 vi = 0; vi < load_sensors_.size(); ++vi )
			network_.set_value( load_group_, vi, snel::real( load_sensors_[ vi ].GetValue() ) );

		network_.update();

		if ( timestamp == 0.0 )
			for ( int i = 0; i < neuron_equilibration_steps; ++i )
				network_.update();

		for ( uint32 mi = 0; mi < muscles_.size(); ++mi )
			actuators_[ mi ].AddInput( network_.value( mn_group_, mi ) );

		return false;
	}

	uint32 SpinalController::AddNeuron( group_id gid, const String& name, Real bias )
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		neuron_names_.emplace_back( neuron_group_names_[ gid.idx ] + '.' + name );
		auto nid = network_.add_neuron( gid, snel::real( bias ) );
		return nid.idx - network_.groups_[ gid.idx ].neuron_begin_.idx;
	}

	uint32 SpinalController::AddNeuron( group_id gid, const String& name, Params& par, const PropNode& pn, const PropNode* pn2 )
	{
		SCONE_ASSERT( network_.neuron_count() == neuron_names_.size() );
		neuron_names_.emplace_back( neuron_group_names_[ gid.idx ] + '.' + name );
		auto* par_pn = TryGetPropNode( neuron_group_names_[ gid.idx ] + "_bias", pn, pn2 );
		auto bias = par_pn ? par.get( GetNameNoSide( neuron_names_.back() ), *par_pn ) : 0.0;
		auto nid = network_.add_neuron( gid, snel::real( bias ) );
		return nid.idx - network_.groups_[ gid.idx ].neuron_begin_.idx;
	}

	group_id SpinalController::AddNeuronGroup( const String& name, const PropNode& pn )
	{
		neuron_group_names_.emplace_back( name );
		return network_.add_group( snel::get_update_fn( pn.get<string>( name + "_activation", activation_ ) ) );
	}

	group_id SpinalController::AddInputNeuronGroup( const String& name )
	{
		neuron_group_names_.emplace_back( name );
		return network_.add_group( snel::no_update );
	}

	group_id SpinalController::AddMuscleGroupNeurons( String name, const PropNode& pn, Params& par )
	{
		auto gid = AddNeuronGroup( name, pn );
		for ( auto& mg : muscle_groups_ )
			AddNeuron( gid, mg.sided_name(), par, pn, &mg.pn_ );
		return gid;
	}

	void SpinalController::Connect( group_id sgid, uint32 sidx, group_id tgid, uint32 tidx, Real weight )
	{
		network_.connect( sgid, sidx, tgid, tidx, snel::real( weight ) );
	}

	void SpinalController::Connect( group_id sgid, uint32 sidx, group_id tgid, uint32 tidx, Params& par, const PropNode& par_pn, size_t size )
	{
		SCONE_ASSERT( size > 0 );
		auto pname = ParName( NeuronName( sgid, sidx ), NeuronName( tgid, tidx ) );
		auto weight = ( 1.0 / Real( size ) ) * par.get( pname, par_pn );
		Connect( sgid, sidx, tgid, tidx, weight );
	}

	void SpinalController::Connect( group_id sgid, uint32 sidx, group_id tgid, uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix )
	{
		const PropNode& par_pn = GetPropNode( sgid, tgid, pn, pn2, suffix );
		Connect( sgid, sidx, tgid, tidx, par, par_pn, 1 );
	}

	void SpinalController::Connect( group_id sgid, const std::vector<xo::uint32>& sidxvec, group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix )
	{
		const PropNode& par_pn = GetPropNode( sgid, tgid, pn, pn2, suffix );
		for ( auto sidx : sidxvec )
			Connect( sgid, sidx, tgid, tidx, par, par_pn, sidxvec.size() );
	}

	void SpinalController::TryConnect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix )
	{
		if ( auto* par_pn = TryGetPropNode( PropNodeName( sgid, tgid, suffix ), pn, pn2 ) )
			Connect( sgid, sidx, tgid, tidx, par, *par_pn, 1 );
	}

	void SpinalController::TryConnect( snel::group_id sgid, const std::vector<xo::uint32>& sidxvec, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix )
	{
		if ( auto* par_pn = TryGetPropNode( PropNodeName( sgid, tgid, suffix ), pn, pn2 ) )
			for ( auto sidx : sidxvec )
				Connect( sgid, sidx, tgid, tidx, par, *par_pn, sidxvec.size() );
	}

	void SpinalController::InitMuscleInfo( const PropNode& pn, Model& model, const Location& loc )
	{
		// setup muscle group list
		for ( auto& [key, mgpn] : pn.select( "MuscleGroup" ) )
			for ( auto side : get_sides( loc ) )
				auto& mg = muscle_groups_.emplace_back( mgpn, side );

		// setup muscle info list
		for ( auto& mus : model.GetMuscles() ) {
			if ( loc.GetSide() == Side::None || loc.GetSide() == mus->GetSide() ) {
				auto& musinf = muscles_.emplace_back( mus->GetName(), xo::index_of( mus, model.GetMuscles() ), NeuralDelay( *mus ) );

				// add to muscle groups
				for ( auto& mg : muscle_groups_ ) {
					if ( mg.side_ == musinf.side_ && mg.muscle_pat_.match( musinf.name_ ) ) {
						mg.muscle_indices_.emplace_back( uint32( muscles_.size() - 1 ) );
						musinf.group_indices_.insert( uint32( xo::index_of( mg, muscle_groups_ ) ) );
					}
				}
				// remove if muscle does not belong to any group
				if ( muscles_.back().group_indices_.empty() )
					muscles_.pop_back();
			}
		}

		for ( auto& mg : muscle_groups_ )
			if ( mg.muscle_indices_.empty() )
				log::warning( mg.name_, " does not contain any muscles" );

		// set muscle group indices
		for ( uint32 mgi = 0; mgi < muscle_groups_.size(); ++mgi ) {
			auto& mg = muscle_groups_[ mgi ];
			auto apat = mg.pn_.try_get<xo::pattern_matcher>( "antagonists" );
			auto rpat = mg.pn_.try_get<xo::pattern_matcher>( "related" );
			auto cl_apat = mg.pn_.try_get<xo::pattern_matcher>( "cl_antagonists" );
			for ( uint32 mgi_other = 0; mgi_other < muscle_groups_.size(); ++mgi_other ) {
				if ( mgi != mgi_other ) {
					auto& mg_other = muscle_groups_[ mgi_other ];
					bool same_side = mg.side_ == mg_other.side_;
					bool is_antagonist = ( same_side && apat && apat->match( mg_other.name_ ) ) || ( !same_side && cl_apat && cl_apat->match( mg_other.name_ ) );
					bool is_related = same_side && rpat && rpat->match( mg_other.name_ );

					if ( !same_side && mg.name_ == mg_other.name_ )
						mg.contra_group_index_ = mgi_other;

					if ( is_antagonist ) {
						mg.ant_group_indices_.emplace_back( mgi_other );
						for ( auto mi : mg.muscle_indices_ )
							muscles_[ mi ].ant_group_indices_.insert( mgi_other );
					}
					if ( is_related )
						mg.related_group_indices_.emplace_back( mgi_other );
				}
			}
		}

		for ( auto& m : muscles_ )
			if ( m.group_indices_.empty() )
				log::warning( m.name_, " is not part of any MuscleGroup" );
	}

	TimeInSeconds SpinalController::NeuralDelay( const Muscle& m ) const
	{
		auto mid = MuscleId( m.GetName() );
		if ( auto it = neural_delays_.find( mid.base_ ); it != neural_delays_.end() )
			return it->second;
		if ( auto it = m.GetModel().neural_delays.find( mid.base_ ); it != m.GetModel().neural_delays.end() )
			return it->second;
		SCONE_ERROR( "Could not find neural delay for " + m.GetName() );
	}

	const PropNode* SpinalController::TryGetPropNode( const string& name, const PropNode& pn, const PropNode* pn2 ) const
	{
		if ( pn2 )
			if ( auto* par_pn = pn2->try_get_child( name ) )
				return par_pn;
		if ( auto* par_pn = pn.try_get_child( name ) )
			return par_pn;
		return nullptr;
	}

	const PropNode& SpinalController::GetPropNode( group_id sgid, group_id tgid, const PropNode& pn, const PropNode* pn2, const char* suffix ) const
	{
		if ( auto* par_pn = TryGetPropNode( PropNodeName( sgid, tgid, suffix ), pn, pn2 ) )
			return *par_pn;
		SCONE_ERROR( "Could not find " + PropNodeName( sgid, tgid, suffix ) );
	}

	string SpinalController::ParName( const string& src, const string& trg ) const
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
				const auto& l = network_.links_[ lid.idx ];
				auto v = network_.value( l.input_ ) * l.weight_;
				frame[ neuron_names_[ i ] + '-' + neuron_names_[ l.input_.idx ] ] = v;
			}
		}
	}

	String SpinalController::GetClassSignature() const {
		String str;
		if ( ia_group_ ) str += 'A';
		if ( ib_group_ || ibe_group_ || ibi_group_ ) str += 'B';
		if ( pm_group_ ) str += 'P';
		if ( rc_group_ ) str += 'R';
		if ( load_group_ ) str += 'L';
		if ( ves_group_ ) str += 'V';
		str += stringf( "-%d-%d", network_.neuron_count(), network_.link_count() );
		return str;
	}

	PropNode SpinalController::GetInfo() const {
		PropNode pn;
		pn[ "neurons" ] = network_.neuron_count();
		pn[ "links" ] = network_.link_count();
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
		for ( auto gid = group_id( 0 ); gid.idx < network_.groups_.size(); ++( gid.idx ) ) {
			auto& gpn = nspn.add_child( neuron_group_names_[ gid.idx ] );
			for ( uint32 nidx = 0; nidx < network_.group_size( group_id( gid ) ); ++nidx ) {
				auto nid = network_.get_id( gid, nidx );
				auto& n = network_.neurons_[ nid.idx ];
				auto& npn = gpn.add_child( neuron_names_[ nid.idx ] );
				npn[ "bias" ] = n.bias_;
				for ( auto lit = n.input_begin_.iter( network_.links_ ); lit != n.input_end_.iter( network_.links_ ); ++lit )
					npn[ neuron_names_[ lit->input_.idx ] ] = lit->weight_;
			}
		}
		return pn;
	}
}
