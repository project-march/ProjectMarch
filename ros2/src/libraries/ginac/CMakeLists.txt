cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

project(GiNaC)
file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/ginac/version.h _ginac_vinfo REGEX "^#define[\t ]+GINACLIB_.*_VERSION.*")
string(REGEX REPLACE "^.*GINACLIB_MAJOR_VERSION[ \t]+([0-9]+).*" "\\1" GINAC_MAJOR_VERSION "${_ginac_vinfo}")
string(REGEX REPLACE "^.*GINACLIB_MINOR_VERSION[ \t]+([0-9]+).*" "\\1" GINAC_MINOR_VERSION "${_ginac_vinfo}")
string(REGEX REPLACE "^.*GINACLIB_MICRO_VERSION[ \t]+([0-9]+).*" "\\1" GINAC_MICRO_VERSION "${_ginac_vinfo}")
set(GINAC_VERSION "${GINAC_MAJOR_VERSION}.${GINAC_MINOR_VERSION}.${GINAC_MICRO_VERSION}")

# Library versioning info
file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/ginac/version.h _ginac_vinfo REGEX "^#define[\t ]+GINAC_LT_.*")
string(REGEX REPLACE "^.*GINAC_LT_CURRENT[ \t]+([0-9]+).*" "\\1" ginac_lt_current "${_ginac_vinfo}")
string(REGEX REPLACE "^.*GINAC_LT_AGE[ \t]+([0-9]+).*" "\\1" ginac_lt_age "${_ginac_vinfo}")
string(REGEX REPLACE "^.*GINAC_LT_REVISION[ \t]+([0-9]+).*" "\\1" ginac_lt_revision "${_ginac_vinfo}")
# XXX: it looks like we need to set this for every platform?
math(EXPR ginaclib_soversion "${ginac_lt_current} - ${ginac_lt_age}")
set(ginaclib_version ${ginaclib_soversion}.${ginac_lt_age}.${ginac_lt_revision})

include(GNUInstallDirs)

# make check
enable_testing()
if (NOT TARGET check)
	add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
endif()
if (NOT TARGET test_suite)
	add_custom_target(test_suite)
endif()
if (WIN32)
	if (NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)
		set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
	endif()
endif()
# make info
if (NOT TARGET info)
	add_custom_target(info ALL)
endif()
if (NOT TARGET html)
	add_custom_target(html)
endif()
if (NOT TARGET pdf)
	add_custom_target(pdf)
endif()

set (CMAKE_CXX_STANDARD 11)

if (NOT DEFINED CLN_SOURCE_DIR)
	set(CLN_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cln)
endif()
if (EXISTS ${CLN_SOURCE_DIR}/CMakeLists.txt)
	add_subdirectory(${CLN_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/cln)
else()
	find_package(CLN 1.2.2 REQUIRED)
endif()

include(CheckIncludeFile)
check_include_file("unistd.h" HAVE_UNISTD_H)

if (NOT DEFINED BUILD_SHARED_LIBS)
	if (NOT MSVC)
		set(BUILD_SHARED_LIBS true)
	else()
		set(BUILD_SHARED_LIBS false)
	endif()
endif()

# Set proper rpath so tools will Just Work(TM) after make install.
# Also take care to add -Wl,-rpath, stanza into the *.pc file so that
#
# g++ `pkg-config --cflags --libs ginac`
#
# will Just Work (TM), too.
# Distro packagers should use -DCMAKE_INSTALL_RPATH="" to avoid
# setting rpath on installed binaries.

# rpath for making binaries/libraries relocatable
set(_ginac_rpath_reloc "$ORIGIN/../lib")
set(_wl_rpath "${CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG}")

# rpath for the pkg-config meta-data.
set(_ginaclib_rpath "${_wl_rpath}${_ginac_rpath_reloc}")
list(FIND CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_FULL_LIBDIR}" isSystemDir)
if ("${isSystemDir}" STREQUAL "-1")
	list(APPEND _ginaclib_rpath "${_wl_rpath}\${libdir}")
endif()
set(GINACLIB_RPATH)
if (NOT CMAKE_SKIP_RPATH)
	if (_wl_rpath)
		set(GINACLIB_RPATH "${_ginaclib_rpath}")
	endif()
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ginac.pc.cmake ${CMAKE_CURRENT_BINARY_DIR}/ginac.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ginac.pc DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

# rpath for libginac.so itself, ginsh, and friends
set(_ginac_rpath ${_ginac_rpath_reloc})
foreach(_d ${CLN_LIBRARY_DIRS} ${CMAKE_INSTALL_FULL_LIBDIR})
	list(FIND CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES "${_d}" isSystemDir)
	if ("${isSystemDir}" STREQUAL "-1")
		list(APPEND _ginac_rpath "${_d}")
	endif()
endforeach()
list(REMOVE_DUPLICATES _ginac_rpath)
string(REPLACE ";" ":" ginac_rpath "${_ginac_rpath}")

if (NOT DEFINED CMAKE_INSTALL_RPATH_USE_LINK_RPATH)
	set(CMAKE_INSTALL_RPATH_USE_LINK_RPATH TRUE)
endif()
if (NOT DEFINED CMAKE_INSTALL_RPATH)
	set(CMAKE_INSTALL_RPATH ${ginac_rpath})
endif()
if (APPLE AND NOT DEFINED CMAKE_INSTALL_NAME_DIR)
	set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_FULL_LIBDIR})
endif()

list(FIND CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_FULL_LIBDIR}" isSystemDir)
if ("${isSystemDir}" STREQUAL "-1")
	string(REPLACE ":" ";" _install_rpath "${CMAKE_INSTALL_RPATH}")
	list(FIND _install_rpath "${CMAKE_INSTALL_FULL_LIBDIR}" _is_rpath_consistent)
	if ("${_is_rpath_consistent}" STREQUAL "-1")
		message(WARNING "the libginac.so library will be installed into "
			"a non-standard directory (${CMAKE_INSTALL_FULL_LIBDIR}), "
				"however, the rpath (${_install_rpath}) "
				"does not contain that directory. Most likely "
				"things won't work without extra configuration "
				"(tweaking LD_LIBRARY_PATH, /etc/ld.so.conf, etc).")
	endif()
endif()

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/ginac-config-version.cmake
	VERSION ${GINAC_VERSION}
	COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/ginac-config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/ginac-config.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ginac
)

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/ginac-config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/ginac-config-version.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ginac
)

include(FindFLEX)
include(FindBISON)
find_package(BISON)
find_package(FLEX)
find_package(Readline)
if (READLINE_FOUND)
	set(HAVE_LIBREADLINE 1)
	set(HAVE_READLINE_READLINE_H 1)
	set(HAVE_READLINE_HISTORY_H 1)
endif()

find_package(LibDL)
if (LIBDL_FOUND)
	set(HAVE_LIBDL 1)
endif()

find_program(MAKEINFO makeinfo)
find_program(FIG2DEV fig2dev)
find_program(PYTHON python)
if (NOT PYTHON)
	find_program(PYTHON python3)
endif()
if (NOT PYTHON)
	message(FATAL_ERROR "Python version 3 or 2 is required")
endif()


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

if (NOT LIBEXECDIR)
	set (LIBEXECDIR "${CMAKE_INSTALL_PREFIX}/libexec")
endif()

add_subdirectory(ginac)
add_subdirectory(tools)
add_subdirectory(check)
if (BISON_FOUND AND FLEX_FOUND)
	add_subdirectory(ginsh)
endif()
if (MAKEINFO)
	add_subdirectory(doc)
endif()

