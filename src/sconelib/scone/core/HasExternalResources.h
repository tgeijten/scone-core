/*
** HasExternalResources.h
**
** Copyright (C) Thomas Geijtenbeek and contributors. All rights reserved.
**
** This file is part of SCONE. For more information, see http://scone.software.
*/

#pragma once

#include "platform.h"
#include "types.h"
#include "PropNode.h"
#include "xo/filesystem/path.h"
#include <vector>

namespace scone
{
	struct ExternalResource {
		ExternalResource( const path& p, bool copy ) : filename_( p ), copy_to_output_( copy ), pn_ptr_( nullptr ) {};
		ExternalResource( const path& p, PropNode* pn ) : filename_( p ), copy_to_output_( false ), pn_ptr_( pn ) {};
		path filename_;
		bool copy_to_output_ = true;
		const PropNode* pn_ptr_ = nullptr;
	};
	using ExternalResourceVec = std::vector<ExternalResource>;

	class SCONE_API HasExternalResources
	{
	public:
		HasExternalResources();
		virtual ~HasExternalResources();

		const ExternalResourceVec& GetExternalResourceVec() const;

		void AddExternalResource( const path& p, bool copy = true ) const;
		void AddPropNodeResource( const path& p, const PropNode* pn ) const;
		void AddExternalResources( const HasExternalResources& other ) const;
		bool Contains( const path& p ) const;
		bool CopyTo( const path& target, xo::error_code* ec = nullptr ) const;

	protected:
		mutable ExternalResourceVec external_resources_;
	};
}
