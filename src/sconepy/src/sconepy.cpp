#include <pybind11/pybind11.h>

#include "scone/core/version.h"
#include "xo/string/string_cast.h"
#include "scone/core/system_tools.h"

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(sconepy, m) {
    m.def( "data_dir", []() { return scone::GetDataFolder().str(); } );
    m.attr("__version__") = xo::to_str( scone::GetSconeVersion() );
}
