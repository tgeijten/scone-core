#pragma once

#include "Controller.h"
#include "xo/utility/smart_enum.h"
#include "scone/optimization/Params.h"
#include "snel/network.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/DelayBuffer.h"
#include "scone/model/Side.h"
#include "xo/container/flat_set.h"

namespace scone
{
	struct MuscleInfo {
		MuscleInfo( const string& name, index_t idx, TimeInSeconds delay ) :
			name_( name ), side_( GetSideFromName( name ) ), index_( idx ), delay_( delay )
		{}

		string name_;
		Side side_;
		index_t index_;
		TimeInSeconds delay_;
		xo::flat_set<xo::uint32> group_indices_;
		xo::flat_set<xo::uint32> ant_group_indices_;
	};

	struct MuscleGroup {
		MuscleGroup( const PropNode& pn, Side side ) :
			name_( pn.get_str( "name" ) ), side_( side ), contra_group_index_( ~xo::uint32( 0 ) ), pn_( pn ), muscle_pat_( pn.get<xo::pattern_matcher>( "muscles" ) )
		{}

		string name_;
		Side side_;
		std::vector<xo::uint32> muscle_indices_;
		std::vector<xo::uint32> ant_group_indices_;
		std::vector<xo::uint32> related_group_indices_;
		xo::uint32 contra_group_index_;
		const PropNode& pn_;
		xo::pattern_matcher muscle_pat_;
		string sided_name() const { return GetSidedName( name_, side_ ); }
	};

	class SpinalController : public Controller
	{
	public:
		SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc );
		virtual ~SpinalController() = default;

		void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;
		PropNode GetInfo() const override;

		bool planar;
		int neuron_equilibration_steps;

	protected:
		bool ComputeControls( Model& model, double timestamp ) override;
		String GetClassSignature() const override;

	private:
		xo::uint32 AddNeuron( snel::group_id group, const String& name, Real bias );
		xo::uint32 AddNeuron( snel::group_id group, const String& name, Params& par, const PropNode& pn, const PropNode* pn2 = nullptr );
		snel::group_id AddNeuronGroup( const String& name, const PropNode& pn );
		snel::group_id AddInputNeuronGroup( const String& name );
		snel::group_id AddMuscleGroupNeurons( String name, const PropNode& pn, Params& par );

		void Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Real weight );
		void Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& par_pn, size_t size );
		void Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix = "_weight" );
		void Connect( snel::group_id sgid, const std::vector<xo::uint32>& sidxvec, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix = "_weight" );
		void TryConnect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix = "_weight" );
		void TryConnect( snel::group_id sgid, const std::vector<xo::uint32>& sidxvec, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix = "_weight" );

		void InitMuscleInfo( const PropNode& pn, Model& model, const Location& loc );
		TimeInSeconds NeuralDelay( const Muscle& m ) const;
		const string& GroupName( snel::group_id gid ) const { return neuron_group_names_[ gid.idx ]; }
		const string& NeuronName( snel::group_id gid, xo::uint32 idx ) const { return neuron_names_[ network_.get_id( gid, idx ).idx ]; }
		Side NeuronSide( snel::group_id gid, xo::uint32 idx ) const { return GetSideFromName( NeuronName( gid, idx ) ); }
		const PropNode* TryGetPropNode( const string& name, const PropNode& pn, const PropNode* pn2 ) const;
		const PropNode& GetPropNode( snel::group_id sgid, snel::group_id tgid, const PropNode& pn, const PropNode* pn2, const char* suffix ) const;
		string ParName( const string& src, const string& trg ) const;
		string PropNodeName( snel::group_id sgid, snel::group_id tgid, const char* suffix ) const {
			return GroupName( sgid ) + "_" + GroupName( tgid ) + suffix;
		}

		const xo::flat_map<String, TimeInSeconds> neural_delays_;
		string activation_;

		std::vector<MuscleGroup> muscle_groups_;
		std::vector<MuscleInfo> muscles_;

		snel::network network_;
		snel::group_id l_group_, f_group_, ves_group_, load_group_;
		snel::group_id ia_group_, ib_group_, ibi_group_, ibe_group_, cpg_group_, pm_group_, mn_group_, rc_group_;
		Real ves_vel_gain_;
		Real l_bias_;
		std::vector<DelayedSensorValue> l_sensors_;
		std::vector<DelayedSensorValue> f_sensors_;
		std::vector<DelayedSensorValue> ves_sensors_;
		std::vector<DelayedSensorValue> load_sensors_;
		std::vector<DelayedActuatorValue> actuators_;
		std::vector<String> neuron_names_;
		std::vector<String> neuron_group_names_;
	};
}
