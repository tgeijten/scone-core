#pragma once

#include "Controller.h"
#include "xo/utility/smart_enum.h"
#include "scone/optimization/Params.h"
#include "snel/network.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/DelayBuffer.h"
#include "scone/model/Side.h"
#include "xo/container/flat_set.h"
#include <vector>

namespace scone
{
	// we're adding this to the scone namespace to improve readability (sue me)
	using snel::group_id;
	using xo::uint32;

	struct MuscleInfo {
		MuscleInfo( const string& name, index_t idx, TimeInSeconds delay ) :
			name_( name ), side_( GetSideFromName( name ) ), index_( idx ), delay_( delay )
		{}

		string name_;
		Side side_;
		index_t index_;
		TimeInSeconds delay_;
		xo::flat_set<uint32> group_indices_;
		xo::flat_set<uint32> ant_group_indices_;
	};

	struct MuscleGroup {
		MuscleGroup( const PropNode& pn, Side side ) :
			name_( pn.get_str( "name" ) ), side_( side ), contra_group_index_( ~uint32( 0 ) ), pn_( pn ), muscle_pat_( pn.get<xo::pattern_matcher>( "muscles" ) )
		{}

		string name_;
		Side side_;
		std::vector<uint32> muscle_indices_;
		std::vector<uint32> ant_group_indices_;
		std::vector<uint32> related_group_indices_;
		uint32 contra_group_index_;
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
		uint32 AddNeuron( group_id group, const String& name, Real bias );
		uint32 AddNeuron( group_id group, const String& name, Params& par, const PropNode& pn, const PropNode* pn2 = nullptr );
		group_id AddNeuronGroup( const String& name, const PropNode& pn );
		group_id AddInputNeuronGroup( const String& name );
		group_id AddMuscleGroupNeurons( String name, const PropNode& pn, Params& par );

		void Connect( group_id sgid, uint32 sidx, group_id tgid, uint32 tidx, Real weight );
		void Connect( group_id sgid, uint32 sidx, group_id tgid, uint32 tidx, Params& par, const PropNode& par_pn, size_t size );
		void Connect( group_id sgid, uint32 sidx, group_id tgid, uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix = "" );
		void Connect( group_id sgid, const std::vector<uint32>& sidxvec, group_id tgid, uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix = "" );
		void TryConnect( group_id sgid, uint32 sidx, group_id tgid, uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix = "" );
		void TryConnect( group_id sgid, const std::vector<uint32>& sidxvec, group_id tgid, uint32 tidx, Params& par, const PropNode& pn, const PropNode* pn2, const char* suffix = "" );

		void InitMuscleInfo( const PropNode& pn, Model& model, const Location& loc );
		TimeInSeconds NeuralDelay( const Muscle& m ) const;
		const string& GroupName( group_id gid ) const { return neuron_group_names_[gid.idx]; }
		const string& NeuronName( group_id gid, uint32 idx ) const { return neuron_names_[network_.get_id( gid, idx ).idx]; }
		Side NeuronSide( group_id gid, uint32 idx ) const { return GetSideFromName( NeuronName( gid, idx ) ); }
		const PropNode* TryGetPropNode( const string& name, const PropNode& pn, const PropNode* pn2 ) const;
		const PropNode& GetPropNode( group_id sgid, group_id tgid, const PropNode& pn, const PropNode* pn2, const char* suffix ) const;
		string ParName( const string& src, const string& trg ) const;
		string PropNodeName( group_id sgid, group_id tgid, const char* suffix ) const {
			return GroupName( sgid ) + "_" + GroupName( tgid ) + suffix + "_weight";
		}

		const xo::flat_map<String, TimeInSeconds> neural_delays_;
		string activation_;

		std::vector<MuscleGroup> muscle_groups_;
		std::vector<MuscleInfo> muscles_;

		snel::network network_;
		group_id l_group_, f_group_, ves_group_, load_group_;
		group_id ia_group_, ib_group_, ibi_group_, ibe_group_, cpg_group_, pm_group_, mn_group_, rc_group_;
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
