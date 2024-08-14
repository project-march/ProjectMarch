
set(ginac_exams
	exam_paranoia
	exam_heur_gcd
	exam_match
	exam_parser
	exam_numeric
	exam_relational
	exam_powerlaws
	exam_collect
	exam_inifcns
	exam_inifcns_nstdsums
	exam_inifcns_elliptic
	exam_differentiation
	exam_polygcd
	exam_collect_common_factors
	exam_normalization
	exam_sqrfree
	exam_factor
	exam_pseries
	exam_matrices
	exam_lsolve
	exam_indexed
	exam_color
	exam_clifford
	exam_archive
	exam_structure
	exam_misc
	exam_pgcd
	exam_mod_gcd
	exam_real_imag
	exam_chinrem_gcd
	exam_function_exvector
)

set(ginac_checks
	check_numeric
	check_inifcns
	check_matrices
	check_lsolve
	check_cra)

set(ginac_timings
	time_dennyfliegner
	time_gammaseries
	time_vandermonde
	time_toeplitz
	time_lw_A
	time_lw_B
	time_lw_C
	time_lw_D
	time_lw_E
	time_lw_F
	time_lw_G
	time_lw_H
	time_lw_IJKL
	time_lw_M1
	time_lw_M2
	time_lw_N
	time_lw_O
	time_lw_P
	time_lw_Pprime
	time_lw_Q
	time_lw_Qprime
	time_antipode
	time_fateman_expand
	time_uvar_gcd
	time_parser)

macro(add_ginac_test thename)
	if ("${${thename}_sources}" STREQUAL "")
		set(${thename}_sources ${thename}.cpp ${${thename}_extra_src})
	endif()
	add_executable(${thename} EXCLUDE_FROM_ALL ${${thename}_sources})
	target_link_libraries(${thename} ginac::ginac)
	add_dependencies(test_suite ${thename})
	add_dependencies(check ${thename})
	add_test(NAME ${thename} COMMAND $<TARGET_FILE:${thename}>)
endmacro()

macro(add_ginac_timing thename)
	set(${thename}_extra_src timer.cpp randomize_serials.cpp)
	add_ginac_test(${thename})
	target_compile_definitions(${thename} PRIVATE HAVE_CONFIG_H)
endmacro()

set(check_matrices_extra_src genex.cpp)
set(check_lsolve_extra_src genex.cpp)

foreach(exm ${ginac_exams})
	add_ginac_test(${exm})
endforeach()

foreach(chk ${ginac_checks})
	add_ginac_test(${chk})
endforeach()

foreach(tmr ${ginac_timings})
	add_ginac_timing(${tmr})
endforeach()

