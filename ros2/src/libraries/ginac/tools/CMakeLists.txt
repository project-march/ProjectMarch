
add_executable(viewgar viewgar.cpp)
target_link_libraries(viewgar ginac::ginac)
install(TARGETS viewgar RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

if (CMAKE_COMPILER_IS_GNUCC)
	set (CC gcc)
	configure_file (
          "${CMAKE_CURRENT_SOURCE_DIR}/ginac-excompiler.in"
          "${CMAKE_CURRENT_BINARY_DIR}/ginac-excompiler"
	)

	install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/ginac-excompiler DESTINATION "${CMAKE_INSTALL_LIBEXECDIR}")
endif (CMAKE_COMPILER_IS_GNUCC)
