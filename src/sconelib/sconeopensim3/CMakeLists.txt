set(MODEL_OPENSIM_3_FILES
	BodyOpenSim3.cpp
	BodyOpenSim3.h
	ContactForceOpenSim3.h
	ContactForceOpenSim3.cpp
	DofOpenSim3.cpp
	DofOpenSim3.h
	JointOpenSim3.cpp
	JointOpenSim3.h
	ModelOpenSim3.cpp
	ModelOpenSim3.h
	MuscleOpenSim3.cpp
	MuscleOpenSim3.h
	ConstantForce.cpp
	ConstantForce.h
	simbody_tools.h
	platform.h
	sconeopensim3.h
	sconeopensim3.cpp
	StateComponentOpenSim3.h
	StateComponentOpenSim3.cpp
)

find_package( OpenSim 3.3 REQUIRED MODULE )

add_library( sconeopensim3 SHARED ${MODEL_OPENSIM_3_FILES} )

target_include_directories( sconeopensim3 PRIVATE ${OPENSIMSIMBODY_INCLUDE_DIRS} )
target_link_libraries( sconeopensim3 PUBLIC sconelib ${OPENSIMSIMBODY_LIBRARIES} )

# Require C++17 standard
set_target_properties(sconeopensim3 PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

if (MSVC)
	target_precompile_headers(sconeopensim3 PRIVATE <string> <vector> <algorithm> <memory> <limits> <fstream> <OpenSim/OpenSim.h>)
	file (GLOB_RECURSE PRECOMPILED_HEADER_FILES ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake_pch.*)
	source_group("CMakePCH" FILES ${PRECOMPILED_HEADER_FILES})
	source_group("" FILES ${MODEL_OPENSIM_3_FILES})

	target_compile_definitions( sconeopensim3 PRIVATE SCONE_MODEL_OPENSIM_3_EXPORTS )
	target_compile_definitions( sconeopensim3 PRIVATE $<$<BOOL:${SCONE_ENABLE_PROFILER}>:SCONE_ENABLE_XO_PROFILING> )
	target_compile_options( sconeopensim3 PRIVATE "/MP" ) # multithreaded compilation on MSVC

	# Copy DLLs
	SET(DLL_TARGET_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIGURATION>)
	add_custom_command(TARGET sconeopensim3 POST_BUILD COMMENT "Updating DLLs" VERBATIM )

	# Copy all OpenSim3 DLLs (x64 only)
	SET(OPENSIM_DLLS 
		${OPENSIM_LIBRARY_PREFIX}Tools 
		${OPENSIM_LIBRARY_PREFIX}Analyses 
		${OPENSIM_LIBRARY_PREFIX}Actuators 
		${OPENSIM_LIBRARY_PREFIX}Simulation 
		${OPENSIM_LIBRARY_PREFIX}Lepton 
		${OPENSIM_LIBRARY_PREFIX}Common 
		${SIMBODY_LIBRARY_PREFIX}SimTKsimbody
		${SIMBODY_LIBRARY_PREFIX}SimTKmath
		${SIMBODY_LIBRARY_PREFIX}SimTKcommon
		)
	foreach(opensimdll ${OPENSIM_DLLS})
		add_custom_command(TARGET sconeopensim3 POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENSIM_INSTALL_DIR}/bin/${opensimdll}$<$<CONFIG:debug>:_d>.dll ${DLL_TARGET_DIR}
		)
		install(FILES ${OPENSIM_INSTALL_DIR}/bin/${opensimdll}$<$<CONFIG:debug>:_d>.dll DESTINATION bin)
	endforeach()

	# Copy additional DLLs required by OpenSim
	set(PTHREAD pthreadVC2_x64)
	set(LIBGCC libgcc_s_sjlj-1)
	SET(OPENSIM_EXTRA_DLLS libblas liblapack ${LIBGCC} libgfortran-3 libquadmath-0 ${PTHREAD})
	foreach(opensimdll ${OPENSIM_EXTRA_DLLS})
		add_custom_command(TARGET sconeopensim3 POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENSIM_INSTALL_DIR}/bin/${opensimdll}.dll ${DLL_TARGET_DIR}
		)
		install(FILES ${OPENSIM_INSTALL_DIR}/bin/${opensimdll}.dll DESTINATION bin)
	endforeach()
endif()

if(LINUX OR APPLE)
	install(DIRECTORY "${OPENSIM_INSTALL_DIR}/lib/" DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

install(TARGETS sconeopensim3
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
