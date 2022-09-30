cmake_minimum_required(VERSION 3.10.0)

project(scone VERSION 2.1.0)

# To create a folder hierarchy within Visual Studio.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# compilation database for completion on Linux
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies.
# ------------------
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

option(SCONE_OPENSIM_3 "Support OpenSim 3.3" OFF)
option(SCONE_OPENSIM_4 "Support OpenSim 4" OFF)
option(SCONE_LUA "Support for lua scripting" OFF)
option(SCONE_HYFYDY "Support Hyfydy - EXPERIMENTAL" OFF)
option(SCONE_ENABLE_PROFILER "Enable SCONE profiler" ON)
option(SCONE_EXPERIMENTAL_FEATURES "Enable experimental features" OFF)
option(SCONE_PYTHON "Build SconePy Python API" OFF)
option(SCONE_USER_EXTENSIONS "Build sconeuser extension library" OFF)
option(SCONE_CORE_CPACK "Build SCONE installer using CPack" OFF)

# Find TCLAP
find_path( TCLAP_DIR NAMES "tclap/CmdLine.h" PATHS "${CMAKE_CURRENT_SOURCE_DIR}/contrib/tclap-1.2.1/include" )

# Various settings
# ----------------
# Place build products (libraries, executables) in root
# binary (build) directory. Otherwise, they get scattered around
# the build directory and so the dll's aren't next to the executables.
#get_filename_component(CONFIG_NAME "${CMAKE_BINARY_DIR}" NAME)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/${CONFIG_NAME}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Set INSTALL directories
if(WIN32)
	set(CMAKE_INSTALL_BINDIR bin)
	set(CMAKE_INSTALL_LIBDIR bin)
endif()

if (UNIX AND NOT APPLE)
	set(CMAKE_INSTALL_RPATH "\$ORIGIN")
	set(CMAKE_INSTALL_BINDIR bin)
	set(CMAKE_INSTALL_LIBDIR lib)
endif()

# Silence CMake warning on OSX.
set(CMAKE_MACOSX_RPATH ON)

# Build lua if SCONE_LUA is set
if (SCONE_LUA)
	add_subdirectory(contrib/lua-5.3.5)
	find_path( SOL_SINGLE_INCLUDE_DIR NAMES sol/sol.hpp HINTS "${CMAKE_CURRENT_SOURCE_DIR}/submodules/sol2/single/include"
	REQUIRED )
endif()

#
# Add submodules
#
add_subdirectory(submodules/xo)
add_subdirectory(submodules/spot)

#
# Add targets
#
add_subdirectory(src/sconelib)
add_subdirectory(src/sconecmd)
add_subdirectory(src/sconeunittests)

if (SCONE_HYFYDY)
	find_path( SCONE_HYFYDY_DIR "sconehfd/sconehfd/ModelHfd.h" HINTS "${CMAKE_SOURCE_DIR}/../sconehfd" "${CMAKE_SOURCE_DIR}/../../sconehfd" )
	add_subdirectory("${SCONE_HYFYDY_DIR}" "${CMAKE_BINARY_DIR}/sconehfd")
endif()

if (SCONE_PYTHON)
	add_subdirectory(submodules/pybind11)
	add_subdirectory(src/sconepy)
endif()

# Create .sconeroot in build folder
if(MSVC)
	file(WRITE "${CMAKE_BINARY_DIR}/.sconeroot" ${CMAKE_CURRENT_SOURCE_DIR})
endif()

#
# Add tests
#
enable_testing()
if (XO_TEST_ENABLED)
	add_test( xo_test "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/xo_test")
endif()
add_test( sconeunittests "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sconeunittests")

#
# Installation / packaging (CPack)
#
if (SCONE_CORE_CPACK)
	# package any required system libraries
	include(InstallRequiredSystemLibraries)

	# set necessary CPack variables
	set(CPACK_PACKAGE_NAME "scone-core")
	set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
	set(CPACK_PACKAGE_VENDOR "Goatstream")
	set(CPACK_PACKAGE_HOMEPAGE_URL "https://goatstream.com")
	set(CPACK_PACKAGE_DESCRIPTION "A tool for predictive musculoskeletal simulations")
	set(CPACK_PACKAGE_EXECUTABLES "sconecmd;SCONE Command Line Interface")

	# install scenarios
	install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/Examples2" DESTINATION "./scenarios")
	install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/Tutorials2" DESTINATION "./scenarios")
	install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/SconePy" DESTINATION "./scenarios")

	# Windows install/packaging:
	if (WIN32)
		# packaging: use NSIS to package everything into a self-extracting installer
		set(CPACK_PACKAGE_INSTALL_DIRECTORY "scone-core")
		set(CPACK_GENERATOR NSIS)
		set(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/resources/scone_icon_24.ico")

		# set(CPACK_NSIS_HELP_LINK ${CPACK_PACKAGE_HOMEPAGE_URL})
		set(CPACK_NSIS_CONTACT "info@goatstream.com")
		set(CPACK_NSIS_MODIFY_PATH OFF)
		set(CPACK_NSIS_IGNORE_LICENSE_PAGE ON)
		set(CPACK_NSIS_CREATE_ICONS "CreateShortCut '\$SMPROGRAMS\\\\scone-core\\\\sconecmd.lnk' '\$INSTDIR\\\\bin\\\\sconecmd.exe'")
	endif()

	if (UNIX AND NOT APPLE)
	    # packaging: package installation as a DEB
	    set(CPACK_GENERATOR DEB)
	    # set(CPACK_PACKAGING_INSTALL_PREFIX /opt/scone-core)
	    # set(CPACK_DEBIAN_PACKAGE_DEPENDS "libblas3, liblapack3, libstdc++6")
	    set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
	endif()

	# CPack vars etc. now fully configured, so include it
	include(CPack)
	
endif()
