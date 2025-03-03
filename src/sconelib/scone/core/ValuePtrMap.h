#pragma once

#include "types.h"
#include "xo/container/flat_map.h"
#include "Log.h"

namespace scone
{
	template< typename T >
	class ValuePtrMap {
	public:
		void Add( const String& name, T* value_ptr ) {
			values_.insert( { name, value_ptr } );
		}

		int TrySet( const String& name, T value ) {
			if ( auto it = values_.find( name ); it != values_.end() ) {
				*( it->second ) = value;
				return 1;
			}
			else return 0;
		}

		xo::optional<T> TryGet( const String& name ) {
			if ( auto it = values_.find( name ); it != values_.end() )
				return *( it->second );
			else return {};
		}

		size_t GetSize() const { return values_.size(); }

		const auto& GetData() const { return values_; }

		const std::vector<String> GetNames() const  {
			std::vector<String> vec;
			vec.reserve( values_.size() );
			for ( const auto& p : values_ )
				vec.emplace_back( p.first );
			return vec;
		}

	private:
		xo::flat_map<String, Real*> values_;
	};

	using RealPtrMap = ValuePtrMap<Real>;
}
