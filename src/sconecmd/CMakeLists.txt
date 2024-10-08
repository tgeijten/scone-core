add_executable(sconecmd sconecmd.cpp)

set_target_properties(sconecmd PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

target_include_directories(sconecmd PRIVATE ${TCLAP_DIR} )

target_link_libraries(sconecmd sconelib)

if (MSVC)
	target_precompile_headers(sconecmd PRIVATE <string> <vector> <algorithm> <memory> <limits> <fstream>)
	file (GLOB_RECURSE PRECOMPILED_HEADER_FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake_pch.*)
	source_group("CMakePCH" FILES ${PRECOMPILED_HEADER_FILES})
	source_group("" FILES sconecmd.cpp)
endif()

if (SCONE_OPENSIM_3)
	target_link_libraries(sconecmd sconeopensim3)
	target_compile_definitions(sconecmd PRIVATE SCONE_OPENSIM_3)
endif()

if (SCONE_OPENSIM_4)
	target_link_libraries(sconecmd sconeopensim4)
	target_compile_definitions(sconecmd PRIVATE SCONE_OPENSIM_4)
endif()

if (SCONE_HYFYDY)
	target_link_libraries(sconecmd sconehfd)
	target_compile_definitions(sconecmd PRIVATE SCONE_HYFYDY)
	if (HFD_BUILD_PRECISION_DOUBLE)
		target_link_libraries(sconecmd sconehfd_dbl)
		target_compile_definitions(sconecmd PRIVATE SCONE_HYFYDY_DBL)
	endif()
endif()

if (SCONE_LUA)
	target_link_libraries(sconecmd sconelua)
	target_compile_definitions(sconecmd PRIVATE SCONE_LUA)
endif()

if (SCONE_USER_EXTENSIONS)
	target_link_libraries(sconecmd sconeuser)
	target_compile_definitions(sconecmd PRIVATE SCONE_USER_EXTENSIONS)
endif()

if (LINUX)
	set_target_properties(sconecmd PROPERTIES INSTALL_RPATH "\$ORIGIN/../lib")
elseif(APPLE)
	set(CMAKE_MACOSX_RPATH TRUE)
	set_target_properties(sconecmd PROPERTIES INSTALL_RPATH "@executable_path/../lib")
endif()

install(TARGETS sconecmd 
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
