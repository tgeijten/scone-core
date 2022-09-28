file (GLOB SCONE_LUA_FILES "*.h" "*.cpp")

# liblua is build as part of SCONE -- no need for find_package (which doesn't work for debug builds)
# find_package(Lua 5.3.5 EXACT REQUIRED)
# SOL_SINGLE_INCLUDE_DIR is found by SCONE

add_library( sconelua SHARED ${SCONE_LUA_FILES} )
target_link_libraries( sconelua sconelib liblua )
target_include_directories( sconelua PRIVATE ${LUA_INCLUDE_DIR} ${SOL_SINGLE_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties( sconelua PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

if (MSVC)
	target_compile_definitions( sconelua PRIVATE SCONE_LUA_EXPORTS )
	target_compile_definitions( sconelua PRIVATE $<$<BOOL:${SCONE_ENABLE_PROFILER}>:SCONE_ENABLE_XO_PROFILING> )
	target_compile_options( sconelua PRIVATE "/bigobj" "/MP" ) # needed to prevent C1128
endif()

source_group( "" FILES ${SCONE_LUA_FILES} )

install(TARGETS sconelua
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	)
