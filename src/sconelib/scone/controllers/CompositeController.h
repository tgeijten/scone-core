/*
** CompositeController.h
**
** Copyright (C) 2013-2018 Thomas Geijtenbeek. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/controllers/Controller.h"

namespace scone
{
	/// Controller consisting of multiple child controllers.
	/// Children can be inserted through an additional 'Controller' tag.
	class CompositeController : public Controller
	{
	public:
		CompositeController( const PropNode& props, Params& par, Model& model, const Location& loc );
		virtual ~CompositeController() {}
		
		virtual bool PerformAnalysis( const Model& model, double timestamp ) override;
		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;
		virtual std::vector<xo::path> WriteResults( const xo::path& file ) const override;

		/// Child node containing all Measures.
		const PropNode* Controllers;

	protected:
		virtual bool ComputeControls( Model& model, double timestamp ) override;

		virtual String GetClassSignature() const override;
		std::vector< ControllerUP > controllers_;
	};
}
