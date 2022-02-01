#pragma once

#include "Controller.h"
#include "xo/utility/smart_enum.h"
#include "scone/optimization/Params.h"
#include "snel/network.h"
#include "xo/container/prop_node_tools.h"
#include "scone/model/DelayBuffer.h"

namespace scone
{
	class SpinalController;

	struct MuscleGroup {
		MuscleGroup() = default;
		MuscleGroup( const PropNode& pn );
		string name_;
		xo::pattern_matcher muscles_;
		std::vector<string> antagonists_;
		std::vector<string> synergists_;
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
		TimeInSeconds GetNeuralDelay( const Muscle& m ) const;
		const xo::flat_map<String, TimeInSeconds> neural_delays_;

		std::vector<MuscleGroup> muscle_groups_;

		snel::network network_;
		snel::group_id spindle_group_, motor_group_, ia_group_;
		std::vector<DelayedSensorValue> delayed_spindle_sensors_;
		std::vector<DelayedActuatorValue> delayed_actuators_;
	};
}

XO_DEFINE_FROM_PROP_NODE_FOR_TYPE( scone::MuscleGroup );
XO_DEFINE_FROM_PROP_NODE_FOR_TYPE( scone::NeuronGroup );
