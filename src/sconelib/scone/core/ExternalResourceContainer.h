#pragma once

#include "platform.h"
#include "types.h"
#include "PropNode.h"
#include "xo/filesystem/path.h"
#include <vector>

namespace scone
{
	struct ExternalResource {
		path filename_;
		bool copy_to_output_ = true;
		const PropNode* pn_ptr_ = nullptr;
	};

	class SCONE_API ExternalResourceContainer
	{
	public:
		void Add( const xo::path& p, bool copy = true );
		void Add( const path& p, const PropNode* pn );
		void Add( const path& f, const PropNode* pn, const std::vector<path>& included_files );
		void Add( const ExternalResourceContainer& other );

		bool Contains( const path& p ) const;
		bool IsEmpty() const { return data_.empty(); }
		bool WriteTo( const path& target_dir, xo::error_code* ec = nullptr ) const;
		const std::vector<ExternalResource>& GetVec() const { return data_; }

	private:
		std::vector<ExternalResource> data_;
	};
}
