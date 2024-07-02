
set(ginaclib_sources
    add.cpp
    archive.cpp
    basic.cpp
    clifford.cpp
    color.cpp
    constant.cpp
    excompiler.cpp
    ex.cpp
    expair.cpp
    expairseq.cpp
    exprseq.cpp
    factor.cpp
    fail.cpp
    fderivative.cpp
    function.cpp
    idx.cpp
    indexed.cpp
    inifcns.cpp
    inifcns_gamma.cpp
    inifcns_nstdsums.cpp
    inifcns_elliptic.cpp
    inifcns_trans.cpp
    integration_kernel.cpp
    integral.cpp
    lst.cpp
    matrix.cpp
    mul.cpp
    ncmul.cpp
    normal.cpp
    numeric.cpp
    operators.cpp
    parser/default_reader.cpp
    parser/lexer.cpp
    parser/parse_binop_rhs.cpp
    parser/parse_context.cpp
    parser/parser_compat.cpp
    parser/parser.cpp
    polynomial/chinrem_gcd.cpp
    polynomial/collect_vargs.cpp
    polynomial/cra_garner.cpp
    polynomial/divide_in_z_p.cpp
    polynomial/gcd_uvar.cpp
    polynomial/mgcd.cpp
    polynomial/mod_gcd.cpp
    polynomial/normalize.cpp
    polynomial/optimal_vars_finder.cpp
    polynomial/pgcd.cpp
    polynomial/primpart_content.cpp
    polynomial/remainder.cpp
    polynomial/upoly_io.cpp
    power.cpp
    print.cpp
    pseries.cpp
    registrar.cpp
    relational.cpp
    remember.cpp
    symbol.cpp
    symmetry.cpp
    tensor.cpp
    utils.cpp
    wildcard.cpp
)

set(ginaclib_public_headers
    ginac.h
    add.h
    archive.h
    assertion.h
    basic.h
    class_info.h
    clifford.h
    color.h
    compiler.h
    constant.h
    container.h
    ex.h
    excompiler.h
    expair.h
    expairseq.h 
    exprseq.h
    fail.h
    factor.h
    fderivative.h
    flags.h
    ${CMAKE_CURRENT_BINARY_DIR}/function.h
    hash_map.h
    idx.h
    indexed.h 
    inifcns.h
    integral.h
    integration_kernel.h
    lst.h
    matrix.h
    mul.h
    ncmul.h
    normal.h
    numeric.h
    operators.h 
    power.h
    print.h
    pseries.h
    ptr.h
    registrar.h
    relational.h
    structure.h 
    symbol.h
    symmetry.h
    tensor.h
    version.h
    wildcard.h 
    parser/parser.h 
    parser/parse_context.h
)

set(ginaclib_private_headers
    remember.h
    utils.h
    crc32.h
    hash_seed.h
    utils_multi_iterator.h
    parser/lexer.h
    parser/debug.h
    polynomial/gcd_euclid.h
    polynomial/remainder.h
    polynomial/normalize.h
    polynomial/upoly.h
    polynomial/ring_traits.h
    polynomial/mod_gcd.h
    polynomial/cra_garner.h
    polynomial/upoly_io.h
    polynomial/prem_uvar.h
    polynomial/eval_uvar.h
    polynomial/interpolate_padic_uvar.h
    polynomial/sr_gcd_uvar.h
    polynomial/heur_gcd_uvar.h
    polynomial/chinrem_gcd.h
    polynomial/collect_vargs.h
    polynomial/divide_in_z_p.h
    polynomial/euclid_gcd_wrap.h
    polynomial/eval_point_finder.h
    polynomial/newton_interpolate.h
    polynomial/optimal_vars_finder.h
    polynomial/pgcd.h
    polynomial/poly_cra.h
    polynomial/primes_factory.h
    polynomial/smod_helpers.h
    polynomial/debug.h
)

add_library(ginac ${ginaclib_sources})
add_library(ginac::ginac ALIAS ginac)
set_target_properties(ginac PROPERTIES
	SOVERSION ${ginaclib_soversion}
	VERSION ${ginaclib_version})
target_compile_definitions(ginac
	PUBLIC $<BUILD_INTERFACE:IN_GINAC>
	PRIVATE -DLIBEXECDIR="${LIBEXECDIR}/" HAVE_CONFIG_H
)
target_link_libraries(ginac PUBLIC cln::cln ${LIBDL_LIBRARIES})
target_include_directories(ginac PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if (WIN32 AND CMAKE_COMPILER_IS_GNUCXX AND BUILD_SHARED_LIBS)
	set_target_properties(ginac PROPERTIES
		LINK_FLAGS "-Wl,--enable-auto-import -Wl,--export-all-symbols"
	)
endif()

if (NOT BUILD_SHARED_LIBS)
	set_target_properties(ginac PROPERTIES OUTPUT_NAME "ginac")
	# Avoid the static library vs import library conflict (necessary for
	# m$ toolchain).
	set_target_properties(ginac PROPERTIES PREFIX "lib")
endif()

install(TARGETS ginac
		      EXPORT ginac-exports
	              LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
		      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
		      ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
install(FILES ${ginaclib_public_headers} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ginac")
install(EXPORT ginac-exports
	FILE ginac-targets.cmake
	NAMESPACE ginac::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ginac
)

add_custom_command(
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/function.h
	COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/function.py -o ${CMAKE_CURRENT_BINARY_DIR}/function.h ${CMAKE_CURRENT_SOURCE_DIR}/function.hppy
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/function.hppy ${CMAKE_CURRENT_SOURCE_DIR}/function.py)

add_custom_command(
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/function.cpp
	COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/function.py -o ${CMAKE_CURRENT_BINARY_DIR}/function.cpp ${CMAKE_CURRENT_SOURCE_DIR}/function.cppy
	DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/function.h ${CMAKE_CURRENT_SOURCE_DIR}/function.cppy ${CMAKE_CURRENT_SOURCE_DIR}/function.py)

