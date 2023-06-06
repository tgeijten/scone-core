#include "system_tests.h"

#include <random>
#include "xo/utility/hash.h"
#include "xo/system/log.h"

namespace scone
{
	int test_rng() {
		auto re = std::minstd_rand( 123 );
		auto rud = std::uniform_real_distribution( -1.0, 1.0 );
		auto rnd = std::normal_distribution( 1.0, 2.0 );
		for ( int i = 0; i < 5; ++i ) {
			printf( "%ld\t%f\t%f\n", re(), rud( re ), rnd( re ) );
		}
		return 0;
	}

	int perform_test( const String& test )
	{
		switch ( xo::hash( test ) ) {
		case "rng"_hash: return test_rng();
		default:
			xo::log::error( "Unknown test: ", test );
			return 0;
		}
	}
}
