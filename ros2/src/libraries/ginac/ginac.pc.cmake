prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_FULL_LIBDIR@
includedir=@CMAKE_INSTALL_FULL_INCLUDEDIR@

Name: GiNaC
Description: C++ library for symbolic calculations
Version: @GINAC_VERSION@
Requires: cln >= 1.2.2
Libs: -L${libdir} -lginac @GINACLIB_RPATH@
Cflags: -I${includedir}
