#pragma once

#include "Controller.h"

namespace scone
{
	struct MuscleGroup {
		string name_;
		std::vector<Muscle*> muscles_;
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
