#pragma once

#include "Controller.h"
#include "xo/utility/smart_enum.h"
#include "scone/optimization/Params.h"
#include "snel/network.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/DelayBuffer.h"
#include "scone/model/Side.h"

namespace scone
{
	class SpinalController;

	struct MuscleInfo {
		MuscleInfo( const string& name ) : name_( name ), side_( GetSideFromName( name ) ) {}

		string name_;
		Side side_;
		std::vector<xo::uint32> group_indices_;
		std::vector<xo::uint32> ant_group_indices_;
	};

	struct MuscleGroup {
		MuscleGroup( const PropNode& pn, Side side ) : pn_( pn ), name_( pn.get_str( "name" ) ), side_( side ) {}

		string name_;
		Side side_;
		std::vector<xo::uint32> muscle_indices_;
		std::vector<xo::uint32> ant_group_indices_;
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

	protected:
		bool ComputeControls( Model& model, double timestamp ) override;
		String GetClassSignature() const override;

	private:
		void InitMuscleInfo( const PropNode& pn, Model& model );
		TimeInSeconds GetNeuralDelay( const Muscle& m ) const;
		snel::neuron_id AddNeuron( snel::group_id group, String name, Real bias );
		snel::neuron_id AddNeuron( snel::group_id group, String name, Params& par, const PropNode& pn, const string& pinf );
		snel::group_id AddMuscleNeurons( String name, const PropNode& pn, Params& par );
		snel::group_id AddGroupNeurons( String name, const PropNode& pn, Params& par );
		snel::group_id AddInterNeurons( String name, size_t count, const PropNode& pn, Params& par );
		snel::link_id Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Real weight );
		snel::link_id Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Params& par, const PropNode& pn, const string& pinf, size_t size );

		const xo::flat_map<String, TimeInSeconds> neural_delays_;
		string activation_;

		std::vector<MuscleGroup> muscle_groups_;
		std::vector<MuscleInfo> muscles_;

		snel::network network_;
		snel::group_id l_group_, ves_group_, mn_group_;
		std::vector<DelayedSensorValue> l_sensors_;
		std::vector<DelayedSensorValue> ves_sensors_;
		std::vector<DelayedActuatorValue> actuators_;
		std::vector<String> neuron_names_;
	};
}
