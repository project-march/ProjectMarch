cmake_minimum_required(VERSION 3.1)

project(GiNaC_examples)

if (TARGET ginac)
	add_info_file(ginac-examples)
endif()

if (NOT TARGET ginac AND EXISTS ${CMAKE_CURRENT_LIST_DIR}/ginac/CMakeLists.txt)
	add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/ginac ${CMAKE_CURRENT_BINARY_DIR}/ginac)
endif()

if (NOT TARGET ginac)
	find_package(ginac REQUIRED 1.7.0)
endif()

add_executable(archive1 archive1.cpp)
target_link_libraries(archive1 ginac::ginac)

add_executable(compile1 compile1.cpp)
target_link_libraries(compile1 ginac::ginac)

# XXX: compile2 example uses the Cuba library (http://www.feynarts.de/cuba)
# add_executable(compile2 compile2.cpp)
# target_link_libraries(compile2 ginac::ginac)

add_executable(compile3 compile3.cpp)
target_link_libraries(compile3 ginac::ginac)

add_executable(mystring mystring.cpp)
target_link_libraries(mystring ginac::ginac)

add_executable(derivative derivative.cpp)
target_link_libraries(derivative ginac::ginac)
