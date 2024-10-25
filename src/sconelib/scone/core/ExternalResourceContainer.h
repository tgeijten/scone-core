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

	class SCONE_API ExternalResourceContainer
	{
	public:
		void Add( const xo::path& p, bool file );
		void Add( const path& p, const PropNode* pn );
		void Add( const ExternalResourceContainer& other );
		bool Contains( const path& p ) const;
		bool WriteTo( const path& target, xo::error_code* ec ) const;
		const std::vector<ExternalResource>& GetVec() const { return external_resources_; }

	private:
		std::vector<ExternalResource> external_resources_;
	};
}
