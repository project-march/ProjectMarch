get_filename_component(ginac_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)
find_package(CLN 1.2.2 REQUIRED)

if (NOT TARGET ginac::ginac)
	include("${ginac_CMAKE_DIR}/ginac-targets.cmake")
endif()

set(ginac_LIBRARIES ginac::ginac)
