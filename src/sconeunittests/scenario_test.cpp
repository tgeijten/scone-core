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
				auto scenario_pn = scone::LoadScenario( test_file_ );
				auto eval_pn = scone::EvaluateScenario( scenario_pn, test_file_, xo::path() );
				auto& eval_result = eval_pn.get_child( "result" ).get_str();

				if ( xo::file_exists( report_file_ ) )
				{
					auto base_pn = xo::load_file( report_file_ );
					auto& base_result = base_pn.get_child( "result" ).get_str();
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
					log::info( eval_pn );
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
	void add_scenario_tests( const path& rootdir, const path& subdir, const xo::pattern_matcher& include, const xo::pattern_matcher& exclude, int levels )
	{
		auto full_test_dir = rootdir / subdir;
		auto results_dir = GetDataFolder() / "resources/UnitTestResults" / subdir / xo::get_computer_name() + "_results";
		if ( XO_IS_DEBUG_BUILD ) results_dir += string( "_debug" );

		xo::log::debug( "scanning dir: ", full_test_dir );

		for ( fs::directory_iterator it( full_test_dir.str() ); it != fs::directory_iterator(); ++it )
		{
			const auto& fs_path = it->path();
			if ( fs::is_directory( fs_path ) && levels > 0 && !exclude( fs_path.filename().string() ) )
				add_scenario_tests( rootdir, subdir / fs_path.filename().string(), include, exclude, levels - 1 );

			auto test_file = xo::path( fs_path.string() );
			auto str = test_file.filename().str();
			if ( include( str ) && !exclude( str ) )
			{
				auto test_name = test_file.parent_path().stem().str() + '/' + test_file.stem().str();
				xo::log::debug( "Adding test case: ", test_name );
				xo::create_directories( results_dir );
				xo::path report_file = results_dir / test_file.stem() + ".zml";
				xo::test::add_test_case( test_name, xo::test::scenario_test( test_file, report_file ) );
			}
		}
	}
}
