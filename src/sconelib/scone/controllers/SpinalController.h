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

	struct MuscleGroup {
		MuscleGroup( const PropNode& pn, const Model& model, Side side, xo::uint32 index );
		void Initialize( const Model& model, const std::vector<MuscleGroup>& muscle_groups );
		string sided_name() const { return GetSidedName( name_, side_ ); }

		const PropNode& pn_;
		string name_;
		Side side_;
		xo::uint32 index_;

		std::vector<xo::uint32> muscle_indices_;
		std::vector<xo::uint32> antaganist_group_indices_;
		std::vector<xo::uint32> antaganist_muscle_indices_;
	};

	xo_smart_enum_class( NeuronGroupType, spindle, force, vestibular, motor, group );
	struct NeuronGroup {
		NeuronGroup() = default;
		NeuronGroup( const PropNode& pn );
		String name_;
		NeuronGroupType type_;
	};

	xo_smart_enum_class( NeuronLinkType, identical, antagonists, synergists );
	xo_smart_enum_class( ActivationType, relu, leaky_relu, tanh, tanh_norm );
	struct NeuronLink {
		NeuronLink( const PropNode& pn, SpinalController& sc );
		string input_;
		string output_;
		NeuronGroupType type_;
		ActivationType activation_;
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
		snel::neuron_id AddNeuron( snel::group_id group, String name, Real bias );
		snel::link_id Connect( snel::group_id sgid, xo::uint32 sidx, snel::group_id tgid, xo::uint32 tidx, Real weight );

		TimeInSeconds GetNeuralDelay( const Muscle& m ) const;
		const xo::flat_map<String, TimeInSeconds> neural_delays_;

		std::vector<MuscleGroup> muscle_groups_;

		snel::network network_;
		snel::group_id spindle_group_, motor_group_;
		std::vector<DelayedSensorValue> delayed_spindle_sensors_;
		std::vector<DelayedActuatorValue> delayed_actuators_;
		std::vector<String> neuron_names_;
	};
}

XO_DEFINE_FROM_PROP_NODE_FOR_TYPE( scone::NeuronGroup );
