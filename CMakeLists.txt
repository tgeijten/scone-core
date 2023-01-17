cmake_minimum_required(VERSION 3.10.0)

project(scone VERSION 2.1.0)

# Set LINUX variable
if(UNIX AND NOT APPLE)
	set(LINUX TRUE)
endif()

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
option(SCONE_DEVELOPMENT_BUILD "Set the SCONE version to be the development version" OFF)
option(SCONE_EXPERIMENTAL_FEATURES "Enable experimental features" OFF)
option(SCONE_PYTHON "Build SconePy Python API" OFF)
option(SCONE_BENCH "Build SCONE Benchmark tool" OFF)
option(SCONE_USER_EXTENSIONS "Build sconeuser extension library" OFF)
option(SCONE_CORE_CPACK "Build SCONE installer using CPack" OFF)
if (LINUX)
	option(SCONE_CORE_CPACK_DEBIAN "Build debian installer using CPack" OFF)
endif()

if(SCONE_CORE_CPACK)
	file (STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/.version" SCONE_REPOSITORY_REVISION)	
	set(CPACK_PACKAGE_VERSION "${CMAKE_PROJECT_VERSION}.${SCONE_REPOSITORY_REVISION}")
endif()

# Find TCLAP
find_path( TCLAP_DIR NAMES "tclap/CmdLine.h" PATHS "${CMAKE_CURRENT_SOURCE_DIR}/contrib/tclap-1.2.1/include" )

# Various settings
# ----------------
# Place build products (libraries, executables) in root
# binary (build) directory. Otherwise, they get scattered around
# the build directory and so the dll's aren't next to the executables.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# set CMAKE_INSTALL_BINDIR etc.
include(GNUInstallDirs)

# Set INSTALL directories
if(LINUX)
	set(CMAKE_INSTALL_RPATH "\$ORIGIN")
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

if (SCONE_BENCH)
	add_subdirectory(src/sconebench)
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
	add_test( NAME xo_test COMMAND xo_test)
endif()
add_test(NAME sconeunittests COMMAND sconeunittests WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

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
	set(CPACK_PACKAGE_CONTACT "info@goatstream.com")
	set(CPACK_PACKAGE_HOMEPAGE_URL "https://goatstream.com")
	set(CPACK_PACKAGE_DESCRIPTION "A tool for predictive musculoskeletal simulations")
	set(CPACK_PACKAGE_EXECUTABLES "sconecmd;SCONE Command Line Interface")
	set(CPACK_OUTPUT_FILE_PREFIX "./../packages")

	# install scenarios
	install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/Examples2" DESTINATION "./scenarios")
	install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/Tutorials2" DESTINATION "./scenarios")
	install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/SconePy" DESTINATION "./scenarios")

	# Windows install/packaging:
	if (WIN32)
		# packaging: use NSIS to package everything into a self-extracting installer
		set(CPACK_PACKAGE_INSTALL_DIRECTORY "scone-core")
		set(CPACK_GENERATOR NSIS)
		set(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/resources/package/scone_icon_24.ico")

		# set(CPACK_NSIS_HELP_LINK ${CPACK_PACKAGE_HOMEPAGE_URL})
		set(CPACK_NSIS_CONTACT "info@goatstream.com")
		set(CPACK_NSIS_MODIFY_PATH OFF)
		set(CPACK_NSIS_IGNORE_LICENSE_PAGE ON)
		set(CPACK_NSIS_CREATE_ICONS "CreateShortCut '\$SMPROGRAMS\\\\scone-core\\\\sconecmd.lnk' '\$INSTDIR\\\\bin\\\\sconecmd.exe'")
	endif()

	if (UNIX AND NOT APPLE)
		if (SCONE_CORE_CPACK_DEBIAN)
			# packaging: package installation as a DEB
			set(CPACK_GENERATOR DEB)
			set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/scone-core")
			set(CPACK_DEBIAN_PACKAGE_DEPENDS "libstdc++6")
			set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
			set(CPACK_STRIP_FILES YES)

			# packaging: configure a script that creates a symlink /usr/local/bin/sconecmd --> /opt/osc/bin/sconecmd
			configure_file("${PROJECT_SOURCE_DIR}/resources/package/postinst.in" "postinst" @ONLY)

			# packaging: configure a script that destroys the above symlink on uninstall
			configure_file("${PROJECT_SOURCE_DIR}/resources/package/postrm.in" "postrm" @ONLY)

			# packaging: tell debian packager to use the scripts for postinst and postrm actions
			set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_BINARY_DIR}/postinst;${CMAKE_BINARY_DIR}/postrm")
		else()
			# packaging: package installation as archive
			set(CPACK_GENERATOR TGZ ZIP)
			set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
			set(CPACK_PACKAGING_INSTALL_PREFIX "/scone-core")
			set(CPACK_DEBIAN_PACKAGE_DEPENDS "libstdc++6")
			set(CPACK_STRIP_FILES YES)
		endif()
	endif()

	# CPack vars etc. now fully configured, so include it
	include(CPack)
	
endif()
