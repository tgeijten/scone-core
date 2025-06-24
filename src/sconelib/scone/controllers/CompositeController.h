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
	/// 
	/** Child Controllers are inserted inside the CompositeController section:
	\verbatim
	CompositeController {
		FeedForwardController { ... }
		ReflexController { ... }
	}
	\endverbatim
	*/
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

		/// prefixes to add to child controller parameter names, so that the same controller can easily be used multiple times; default = empty.
		std::vector<String> child_names;

		virtual PropNode GetInfo() const override;

		const std::vector< ControllerUP >& GetChildren() const { return controllers_; }
		std::vector< ControllerUP >& GetChildren() { return controllers_; }
		template< typename T > T* TryGetChild() {
			for ( auto& c : controllers_ )
				if ( auto* ct = dynamic_cast<T*>( c.get() ) )
					return ct;
			return nullptr;
		}
		
		Controller* InsertChildController( ControllerUP child, index_t pos = 0 );

		int TrySetControlParameter( const String& name, Real value ) override;
		xo::optional<Real> TryGetControlParameter( const String& name ) override;
		std::vector<String> GetControlParameters() const override;

	protected:
		virtual bool ComputeControls( Model& model, double timestamp ) override;
		virtual UpdateResult PerformAnalysis( const Model& model, double timestamp ) override;

		virtual String GetClassSignature() const override;
		std::vector< ControllerUP > controllers_;
	};
}
