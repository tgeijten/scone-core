#include "ExternalResourceContainer.h"

#include "xo/container/container_tools.h"
#include "xo/filesystem/filesystem.h"
#include "xo/serialization/serialize.h"

namespace scone
{
	
	void ExternalResourceContainer::Add( const xo::path& p, bool file )
	{
		if ( !Contains( p ) )
			external_resources_.emplace_back( p, file );
	}

	void ExternalResourceContainer::Add( const path& p, const PropNode* pn )
	{
		if ( !Contains( p ) )
			external_resources_.emplace_back( p, pn );
	}

	void ExternalResourceContainer::Add( const ExternalResourceContainer& other )
	{
		for ( const auto& p : other.external_resources_ )
			if ( !Contains( p.filename_ ) )
				external_resources_.emplace_back( p );
	}

	bool ExternalResourceContainer::Contains( const path& p ) const
	{
		return xo::contains_if( external_resources_, [&]( const auto& e ) { return e.filename_ == p; } );
	}

	bool ExternalResourceContainer::WriteTo( const path& target, xo::error_code* ec ) const
	{
		for ( auto& f : external_resources_ ) {
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
