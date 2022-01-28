#pragma once

#include "Controller.h"
#include "xo/utility/smart_enum.h"
#include "scone/optimization/Params.h"

namespace scone
{
	class SpinalController;

	struct MuscleGroup {
		MuscleGroup( const PropNode& pn, SpinalController& sc );
		string name_;
		std::vector<Muscle*> muscles_;
		std::vector<string> antagonists_;
		std::vector<string> synergists_;
		std::vector<string> linked_;
	};

	xo_smart_enum_class( NeuronGroupType, spindle, force, vestibular, motor, group );
	struct NeuronGroup {
		NeuronGroup( const PropNode& pn, SpinalController& sc );
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
		
	protected:
	private:
	};
}
