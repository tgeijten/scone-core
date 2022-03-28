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
	class SpinalController;

	using index_vec = std::vector<xo::uint32>;

	struct MuscleInfo {
		MuscleInfo( const string& name, TimeInSeconds delay ) : name_( name ), side_( GetSideFromName( name ) ), delay_( delay ) {}

		string name_;
		Side side_;
		TimeInSeconds delay_;
		xo::flat_set<xo::uint32> group_indices_;
		xo::flat_set<xo::uint32> ant_group_indices_;
	};

	struct MuscleGroup {
		MuscleGroup( const PropNode& pn, Side side ) : pn_( pn ), name_( pn.get_str( "name" ) ), side_( side ) {}

		string name_;
		Side side_;
		index_vec muscle_indices_;
		index_vec ant_group_indices_;
		index_vec parent_group_indices_;
		const PropNode& pn_;
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

	protected:
		bool ComputeControls( Model& model, double timestamp ) override;
		String GetClassSignature() const override;

	private:
		xo::uint32 AddNeuron( snel::group_id group, const String& name, Real bias );
		xo::uint32 AddNeuron( snel::group_id group, const String& name, const PropNode& pn, Params& par );
		snel::group_id AddNeuronGroup( const String& name, const PropNode& pn );
		snel::group_id AddInputNeuronGroup( const String& name );
		snel::group_id AddMuscleGroupNeurons( String name, const PropNode& pn, Params& par );

		snel::link_id Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Real weight );
		snel::link_id Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const MuscleGroup* mg, size_t size );
		void Connect( snel::group_id sgid, const index_vec& sidxvec, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const MuscleGroup* mg );

		void InitMuscleInfo( const PropNode& pn, Model& model );
		TimeInSeconds GetNeuralDelay( const Muscle& m ) const;
		const string& GroupName( snel::group_id gid ) const { return neuron_group_names_[ gid.value() ]; }
		const string& GetNeuronName( snel::group_id gid, xo::uint32 idx ) const { return neuron_names_[ network_.get_id( gid, idx ).value() ]; }
		Side GetNeuronSide( snel::group_id gid, xo::uint32 idx ) const { return GetSideFromName( GetNeuronName( gid, idx ) ); }
		const string GetParName( const string& src, const string& trg ) const;

		const xo::flat_map<String, TimeInSeconds> neural_delays_;
		string activation_;

		std::vector<MuscleGroup> muscle_groups_;
		std::vector<MuscleInfo> muscles_;

		snel::network network_;
		snel::group_id l_group_, f_group_, ves_group_, load_group_, ia_group_, ib_group_, cpg_group_, mn_group_;
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
