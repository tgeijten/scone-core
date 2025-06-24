#pragma once

#include "scone/core/platform.h"
#include "scone/core/types.h"

namespace scone
{
	struct SCONE_API UpdateResult
	{
		UpdateResult() : must_terminate_( false ), termination_reason_() {}
		UpdateResult( bool must_terminate ) : must_terminate_( must_terminate ), termination_reason_() {}
		UpdateResult( String termination_reason ) : must_terminate_( true ), termination_reason_( std::move( termination_reason ) ) {}

		UpdateResult& operator |=( const UpdateResult& other ) {
			if ( other.must_terminate_ ) {
				must_terminate_ = true;
				xo::append_str( termination_reason_, other.termination_reason_, "\n" );
			}
			return *this;
		}

		bool must_terminate_;
		String termination_reason_;
	};
}
