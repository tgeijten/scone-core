set(SCONE_USER_FILES
	sconeuser.h
	sconeuser.cpp
	# add user Controllers and Measures here
	ExampleController.h
	ExampleController.cpp
)

add_library( sconeuser SHARED ${SCONE_USER_FILES} )

target_link_libraries( sconeuser PUBLIC sconelib )

# Require C++17 standard
set_target_properties(sconeuser PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

if (MSVC)
	target_compile_definitions( sconeuser PRIVATE SCONE_USER_EXPORTS )
	target_compile_definitions( sconeuser PRIVATE $<$<BOOL:${SCONE_ENABLE_PROFILER}>:SCONE_ENABLE_XO_PROFILING> )
	target_compile_options( sconeuser PRIVATE "/MP" ) # multithreaded compilation on MSVC
endif()

source_group("" FILES ${SCONE_USER_FILES})
