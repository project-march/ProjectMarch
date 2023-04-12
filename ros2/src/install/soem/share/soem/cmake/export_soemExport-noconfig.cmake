#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "soem::soem" for configuration ""
set_property(TARGET soem::soem APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(soem::soem PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "pthread;rt"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsoem.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS soem::soem )
list(APPEND _IMPORT_CHECK_FILES_FOR_soem::soem "${_IMPORT_PREFIX}/lib/libsoem.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
