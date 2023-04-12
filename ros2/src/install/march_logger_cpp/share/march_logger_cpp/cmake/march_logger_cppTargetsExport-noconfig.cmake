#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "march_logger_cpp::march_logger_cpp" for configuration ""
set_property(TARGET march_logger_cpp::march_logger_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(march_logger_cpp::march_logger_cpp PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmarch_logger_cpp.so"
  IMPORTED_SONAME_NOCONFIG "libmarch_logger_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS march_logger_cpp::march_logger_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_march_logger_cpp::march_logger_cpp "${_IMPORT_PREFIX}/lib/libmarch_logger_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
