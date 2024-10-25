#include "ExternalResourceContainer.h"

#include "xo/container/container_tools.h"
#include "xo/filesystem/filesystem.h"
#include "xo/serialization/serialize.h"

namespace scone
{

	void ExternalResourceContainer::Add( const xo::path& p, bool copy )
	{
		if ( !Contains( p ) )
			data_.push_back( ExternalResource{ p, copy, nullptr } );
	}

	void ExternalResourceContainer::Add( const path& p, const PropNode* pn )
	{
		if ( !Contains( p ) )
			data_.emplace_back( ExternalResource{ p, false, pn } );
	}

	void ExternalResourceContainer::Add( const ExternalResourceContainer& other )
	{
		for ( const auto& p : other.data_ )
			if ( !Contains( p.filename_ ) )
				data_.emplace_back( p );
	}

	bool ExternalResourceContainer::Contains( const path& p ) const
	{
		return xo::contains_if( data_, [&]( const auto& e ) { return e.filename_ == p; } );
	}

	bool ExternalResourceContainer::WriteTo( const path& target, xo::error_code* ec ) const
	{
		for ( auto& f : data_ ) {
			if ( f.copy_to_output_ ) {
				xo::error_code copy_ec;
				if ( !xo::copy_file( f.filename_, target / f.filename_.filename(), true, &copy_ec ) ) {
					set_error_or_throw( ec, "Could not copy \"" + f.filename_.str() +
						"\" to \"" + ( target / f.filename_.filename() ).str() + "\"\n\n" + copy_ec.message() );
					return false;
				}
			}
			else if ( f.pn_ptr_ ) {
				if ( !xo::save_file( *f.pn_ptr_, target / f.filename_.filename(), ec ) )
					return false;
			}
		}
		return true;
	}

}
