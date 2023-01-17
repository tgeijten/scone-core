add_executable(sconebench sconebench.cpp)

set_target_properties(sconebench PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

target_include_directories(sconebench PRIVATE ${TCLAP_DIR} )

target_link_libraries(sconebench sconelib)

if (MSVC)
	target_precompile_headers(sconebench PRIVATE <string> <vector> <algorithm> <memory> <limits> <fstream>)
	file (GLOB_RECURSE PRECOMPILED_HEADER_FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake_pch.*)
	source_group("CMakePCH" FILES ${PRECOMPILED_HEADER_FILES})
	source_group("" FILES sconebench cpp)
endif()

if (SCONE_OPENSIM_3)
	target_link_libraries(sconebench sconeopensim3)
	target_compile_definitions(sconebench PRIVATE SCONE_OPENSIM_3)
endif()

if (SCONE_OPENSIM_4)
	target_link_libraries(sconebench sconeopensim4)
	target_compile_definitions(sconebench PRIVATE SCONE_OPENSIM_4)
endif()

if (SCONE_HYFYDY)
	target_link_libraries(sconebench sconehfd)
	target_compile_definitions(sconebench PRIVATE SCONE_HYFYDY)
endif()

if (SCONE_LUA)
	target_link_libraries(sconebench sconelua)
	target_compile_definitions(sconebench PRIVATE SCONE_LUA)
endif()

if (SCONE_USER_EXTENSIONS)
	target_link_libraries(sconebench sconeuser)
	target_compile_definitions(sconebench PRIVATE SCONE_USER_EXTENSIONS)
endif()

if (LINUX)
	set_target_properties(sconebench PROPERTIES INSTALL_RPATH "\$ORIGIN/../lib")
elseif(APPLE)
    set_target_properties(sconebench PROPERTIES INSTALL_RPATH "@executable_path/lib")
endif()

install(TARGETS sconebench 
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
