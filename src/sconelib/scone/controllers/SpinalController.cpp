#include "SpinalController.h"

namespace scone
{
	MuscleGroup::MuscleGroup( const PropNode& pn, SpinalController& sc ) :
		INIT_MEMBER_REQUIRED( pn, name_ ),
		INIT_MEMBER( pn, antagonists_, {} ),
		INIT_MEMBER( pn, synergists_, {} ),
		INIT_MEMBER( pn, linked_, {} )
	{}

	NeuronGroup::NeuronGroup( const PropNode& pn, SpinalController& sc ) :
		INIT_MEMBER_REQUIRED( pn, type_ )
	{}

	NeuronLink::NeuronLink( const PropNode& pn, SpinalController& sc ) :
		INIT_MEMBER_REQUIRED( pn, type_ ),
		INIT_MEMBER_REQUIRED( pn, activation_ )
	{}

	SpinalController::SpinalController( const PropNode& pn, Params& par, Model& model, const Location& loc ) :
		Controller( pn, par, model, loc )
	{}
}
