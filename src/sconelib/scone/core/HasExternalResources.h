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
#include "ExternalResourceContainer.h"
#include <vector>

namespace scone
{
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
