#pragma once

#include "types.h"
#include "xo/container/flat_map.h"
#include "Log.h"

namespace scone
{
	template< typename T >
	class ValuePtrMap {
	public:
		void insert( const String& name, T* value_ptr ) {
			values_.insert( { name, value_ptr } );
		}

		bool try_set( const String& name, T value ) {
			if ( auto it = values_.find( name ); it != values_.end() ) {
				*( it->second ) = value;
				return true;
			}
			else return false;
		}

		xo::optional<T> try_get( const String& name ) {
			if ( auto it = values_.find( name ); it != values_.end() )
				return *( it->second );
			else return {};
		}

		size_t size() const { return values_.size(); }

		const auto& data() const { return values_; }

	private:
		xo::flat_map<String, Real*> values_;
	};
}
