/*
** CompositeController.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "scone/controllers/Controller.h"

namespace scone
{
	/// Controller consisting of multiple child controllers.
	/// Child Controllers are inserted as children of this parameter, e.g. ''CompositeController { FeedForwardController { ... } ReflexController { ... } }''.
	class SCONE_API CompositeController : public Controller
	{
	public:
		CompositeController( const PropNode& props, Params& par, Model& model, const Location& loc );
		CompositeController( const CompositeController& props ) = delete;
		CompositeController& operator=( const CompositeController& props ) = delete;
		virtual ~CompositeController() = default;

		virtual void Reset( Model& model ) override;

		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;
		virtual std::vector<xo::path> WriteResults( const xo::path& file ) const override;

		const PropNode* Controllers;

		virtual PropNode GetInfo() const override;

		const std::vector< ControllerUP >& GetChildren() const { return controllers_; }
		std::vector< ControllerUP >& GetChildren() { return controllers_; }

	protected:
		virtual bool ComputeControls( Model& model, double timestamp ) override;
		virtual bool PerformAnalysis( const Model& model, double timestamp ) override;

		virtual String GetClassSignature() const override;
		std::vector< ControllerUP > controllers_;
	};
}
