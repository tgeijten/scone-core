#pragma once

#include "sconepy.h"
#include "scone/core/PropNode.h"
#include "scone/core/Factories.h"
#include "scone/optimization/opt_tools.h"

namespace scone
{
	struct sconepy_scenario {
		sconepy_scenario( const std::string& file, const std::map<std::string, std::string>& args ) {
			file_ = FindScenario( file );
			pn_ = LoadScenario( file_ );
			set_multiple( args );
		}

		void set( const std::string& key, const std::string& value ) {
			pn_.set_query( key, value );
		}

		void set_multiple( const std::map<std::string, std::string> args ) {
			for ( const auto& [key, value] : args )
				set( key, value );
		}

		OptimizerUP create_optimizer() const {
			auto opt_fp = FindFactoryProps( GetOptimizerFactory(), pn_, "Optimizer" );
			return GetOptimizerFactory().create( opt_fp.type(), opt_fp.props(), pn_, file_.parent_path() );
		}

		OptimizerUP start_optimization() const {
			auto opt = create_optimizer();
			opt->RunBackground();
			return opt;
		}

		xo::path file_;
		PropNode pn_;
	};

	sconepy_scenario load_scenario( const std::string& file, const std::map<std::string, std::string>& args = {} ) {
		return sconepy_scenario{ file, args };
	}
}
