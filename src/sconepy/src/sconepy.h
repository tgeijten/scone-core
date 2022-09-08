#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

template< typename C >
py::array to_array( C&& cont, bool use_f32 ) {
	if ( use_f32 )
		return py::array_t<float>( py::cast( std::move( cont ) ) );
	else
		return py::array_t<double>( py::cast( std::move( cont ) ) );
};

template< typename T >
std::pair<py::array_t<T>, T*> make_array( size_t s ) {
	std::pair<py::array_t<T>, T*> r{ py::array_t<T>( s ), nullptr };
	r.second = static_cast<T*>( r.first.request().ptr );
	return r;
};

static inline const std::string g_sensor_user_data_key = "SPDS";
static inline const std::string g_scenario_user_data_key = "SPPN";
