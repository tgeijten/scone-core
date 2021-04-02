#include "scenario_test.h"

#include <filesystem>
#include "scone/core/system_tools.h"
#include "scone/optimization/opt_tools.h"
#include "xo/filesystem/filesystem.h"
#include "xo/filesystem/path.h"
#include "xo/serialization/serialize.h"
#include "xo/system/log.h"
#include "xo/system/test_case.h"

namespace fs = std::filesystem;

namespace xo
{
	namespace test
	{
		struct scenario_test {
			scenario_test( const path& scenario_file, const path& report_file ) :
				test_file_( scenario_file ),
				report_file_( report_file )
			{}

			void operator()( test_case& XO_ACTIVE_TEST_CASE ) {
				auto scenario_file = scone::FindScenario( test_file_ );
				auto scenario_pn = xo::load_file_with_include( scenario_file, "INCLUDE" );
				auto eval_pn = scone::EvaluateScenario( scenario_pn, test_file_, xo::path() );

				if ( xo::file_exists( report_file_ ) )
				{
					auto base_pn = xo::load_file( report_file_ );
					auto& eval_result = eval_pn.get_child( "result" ).raw_value();
					auto& base_result = base_pn.get_child( "result" ).raw_value();
					XO_CHECK_MESSAGE( eval_result == base_result, eval_result + " != " + base_result );
					if ( eval_result != base_result )
					{
						xo::log::debug( "baseline:\n", base_pn );
						xo::log::debug( "evaluated:\n", eval_pn );
					}
				}
				else
				{
					XO_CHECK_MESSAGE( false, "Could not find results for " + report_file_.filename().str() );
					xo::save_file( eval_pn, report_file_ );
				}
			}

			path test_file_;
			path report_file_;
		};
	}
}

namespace scone
{
	void add_scenario_tests( const path& test_dir, const xo::pattern_matcher& include, const xo::pattern_matcher& exclude, bool include_subdirs )
	{
		auto full_test_dir = GetFolder( SCONE_ROOT_FOLDER ) / test_dir;
		auto results_dir = GetFolder( SCONE_ROOT_FOLDER ) / "resources/unittestdata" / test_dir / xo::get_computer_name() + "_results";
		xo::create_directories( results_dir );

		xo::log::debug( "Adding dir: ", full_test_dir );

		for ( fs::directory_iterator it( full_test_dir.str() ); it != fs::directory_iterator(); ++it )
		{
			const auto& fs_path = it->path();
			if ( fs::is_directory( fs_path ) && fs_path.filename() != "data" )
				add_scenario_tests( test_dir / fs_path.filename().string(), include, exclude, include_subdirs );

			auto test_file = xo::path( fs_path.string() );
			auto str = test_file.filename().str();
			if ( include( str ) && !exclude( str ) )
			{
				xo::log::debug( "Adding test case: ", test_file.filename() );
				xo::path report_file = results_dir / test_file.stem() + ".zml";
				xo::test::add_test_case( test_file.stem().str(), xo::test::scenario_test( test_file, report_file ) );
			}
		}
	}
}
