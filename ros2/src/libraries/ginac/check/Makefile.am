## Process this file with automake to produce Makefile.in

EXAMS = exam_paranoia \
	exam_heur_gcd \
	exam_match \
	exam_parser \
	exam_numeric \
	exam_relational \
	exam_powerlaws \
	exam_collect \
	exam_inifcns \
	exam_inifcns_nstdsums \
	exam_inifcns_elliptic \
	exam_differentiation  \
	exam_polygcd \
	exam_collect_common_factors \
	exam_normalization \
	exam_sqrfree  \
	exam_factor \
	exam_pseries  \
	exam_matrices  \
	exam_lsolve  \
	exam_indexed  \
	exam_color \
	exam_clifford  \
	exam_archive  \
	exam_structure  \
	exam_misc \
	exam_pgcd \
	exam_mod_gcd \
	exam_chinrem_gcd \
	exam_function_exvector \
	exam_real_imag

CHECKS = check_numeric \
	 check_inifcns \
	 check_matrices \
	 check_lsolve \
	 check_cra

TIMES = time_dennyfliegner \
	time_gammaseries \
	time_vandermonde \
	time_toeplitz \
	time_lw_A \
	time_lw_B \
	time_lw_C \
	time_lw_D \
	time_lw_E \
	time_lw_F \
	time_lw_G \
	time_lw_H \
	time_lw_IJKL \
	time_lw_M1 \
	time_lw_M2 \
	time_lw_N \
	time_lw_O \
	time_lw_P \
	time_lw_Pprime \
	time_lw_Q \
	time_lw_Qprime \
	time_antipode \
	time_fateman_expand \
	time_uvar_gcd \
	time_parser

TESTS = $(EXAMS) $(CHECKS) $(TIMES)
check_PROGRAMS = $(EXAMS) $(CHECKS) $(TIMES)

exam_paranoia_SOURCES = exam_paranoia.cpp
exam_paranoia_LDADD = ../ginac/libginac.la

exam_heur_gcd_SOURCES = exam_heur_gcd.cpp
exam_heur_gcd_LDADD = ../ginac/libginac.la

exam_match_SOURCES = exam_match.cpp error_report.h
exam_match_LDADD = ../ginac/libginac.la

exam_parser_SOURCES = exam_parser.cpp
exam_parser_LDADD = ../ginac/libginac.la

exam_numeric_SOURCES = exam_numeric.cpp
exam_numeric_LDADD = ../ginac/libginac.la

exam_relational_SOURCES = exam_relational.cpp
exam_relational_LDADD = ../ginac/libginac.la

exam_powerlaws_SOURCES = exam_powerlaws.cpp
exam_powerlaws_LDADD = ../ginac/libginac.la

exam_collect_SOURCES = exam_collect.cpp
exam_collect_LDADD = ../ginac/libginac.la

exam_inifcns_SOURCES = exam_inifcns.cpp
exam_inifcns_LDADD = ../ginac/libginac.la

exam_inifcns_nstdsums_SOURCES = exam_inifcns_nstdsums.cpp \
				exam_inifcns_nstdsums.h
exam_inifcns_nstdsums_LDADD = ../ginac/libginac.la

exam_inifcns_elliptic_SOURCES = exam_inifcns_elliptic.cpp
exam_inifcns_elliptic_LDADD = ../ginac/libginac.la

exam_differentiation_SOURCES = exam_differentiation.cpp
exam_differentiation_LDADD = ../ginac/libginac.la

exam_polygcd_SOURCES = exam_polygcd.cpp
exam_polygcd_LDADD = ../ginac/libginac.la

exam_collect_common_factors_SOURCES = exam_collect_common_factors.cpp
exam_collect_common_factors_LDADD = ../ginac/libginac.la

exam_normalization_SOURCES = exam_normalization.cpp
exam_normalization_LDADD = ../ginac/libginac.la

exam_sqrfree_SOURCES = exam_sqrfree.cpp
exam_sqrfree_LDADD = ../ginac/libginac.la

exam_factor_SOURCES = exam_factor.cpp
exam_factor_LDADD = ../ginac/libginac.la

exam_pseries_SOURCES = exam_pseries.cpp
exam_pseries_LDADD = ../ginac/libginac.la

exam_matrices_SOURCES = exam_matrices.cpp
exam_matrices_LDADD = ../ginac/libginac.la

exam_lsolve_SOURCES = exam_lsolve.cpp
exam_lsolve_LDADD = ../ginac/libginac.la

exam_indexed_SOURCES = exam_indexed.cpp
exam_indexed_LDADD = ../ginac/libginac.la

exam_color_SOURCES = exam_color.cpp
exam_color_LDADD = ../ginac/libginac.la

exam_clifford_SOURCES = exam_clifford.cpp
exam_clifford_LDADD = ../ginac/libginac.la

exam_archive_SOURCES = exam_archive.cpp
exam_archive_LDADD = ../ginac/libginac.la

exam_structure_SOURCES = exam_structure.cpp
exam_structure_LDADD = ../ginac/libginac.la

exam_misc_SOURCES = exam_misc.cpp
exam_misc_LDADD = ../ginac/libginac.la

exam_pgcd_SOURCES = exam_pgcd.cpp
exam_pgcd_LDADD = ../ginac/libginac.la

exam_mod_gcd_SOURCES = exam_mod_gcd.cpp
exam_mod_gcd_LDADD = ../ginac/libginac.la

exam_real_imag_SOURCES = exam_real_imag.cpp
exam_real_imag_LDADD = ../ginac/libginac.la

exam_chinrem_gcd_SOURCES = exam_chinrem_gcd.cpp
exam_chinrem_gcd_LDADD = ../ginac/libginac.la

exam_function_exvector_SOURCES = exam_function_exvector.cpp
exam_function_exvector_LDADD = ../ginac/libginac.la

check_numeric_SOURCES = check_numeric.cpp
check_numeric_LDADD = ../ginac/libginac.la

check_inifcns_SOURCES = check_inifcns.cpp
check_inifcns_LDADD = ../ginac/libginac.la

check_matrices_SOURCES = check_matrices.cpp genex.cpp
check_matrices_LDADD = ../ginac/libginac.la

check_lsolve_SOURCES = check_lsolve.cpp genex.cpp
check_lsolve_LDADD = ../ginac/libginac.la

check_cra_SOURCES = check_cra.cpp
check_cra_LDADD = ../ginac/libginac.la

time_dennyfliegner_SOURCES = time_dennyfliegner.cpp \
			     randomize_serials.cpp timer.cpp timer.h
time_dennyfliegner_LDADD = ../ginac/libginac.la

time_gammaseries_SOURCES = time_gammaseries.cpp \
			   randomize_serials.cpp timer.cpp timer.h
time_gammaseries_LDADD = ../ginac/libginac.la

time_vandermonde_SOURCES = time_vandermonde.cpp \
			   randomize_serials.cpp timer.cpp timer.h
time_vandermonde_LDADD = ../ginac/libginac.la

time_toeplitz_SOURCES = time_toeplitz.cpp \
			randomize_serials.cpp timer.cpp timer.h
time_toeplitz_LDADD = ../ginac/libginac.la

time_lw_A_SOURCES = time_lw_A.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_A_LDADD = ../ginac/libginac.la

time_lw_B_SOURCES = time_lw_B.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_B_LDADD = ../ginac/libginac.la

time_lw_C_SOURCES = time_lw_C.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_C_LDADD = ../ginac/libginac.la

time_lw_D_SOURCES = time_lw_D.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_D_LDADD = ../ginac/libginac.la

time_lw_E_SOURCES = time_lw_E.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_E_LDADD = ../ginac/libginac.la

time_lw_F_SOURCES = time_lw_F.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_F_LDADD = ../ginac/libginac.la

time_lw_G_SOURCES = time_lw_G.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_G_LDADD = ../ginac/libginac.la

time_lw_H_SOURCES = time_lw_H.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_H_LDADD = ../ginac/libginac.la

time_lw_IJKL_SOURCES = time_lw_IJKL.cpp \
		       randomize_serials.cpp timer.cpp timer.h
time_lw_IJKL_LDADD = ../ginac/libginac.la

time_lw_M1_SOURCES = time_lw_M1.cpp \
		     randomize_serials.cpp timer.cpp timer.h
time_lw_M1_LDADD = ../ginac/libginac.la

time_lw_M2_SOURCES = time_lw_M2.cpp \
		     randomize_serials.cpp timer.cpp timer.h
time_lw_M2_LDADD = ../ginac/libginac.la

time_lw_N_SOURCES = time_lw_N.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_N_LDADD = ../ginac/libginac.la

time_lw_O_SOURCES = time_lw_O.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_O_LDADD = ../ginac/libginac.la

time_lw_P_SOURCES = time_lw_P.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_P_LDADD = ../ginac/libginac.la

time_lw_Pprime_SOURCES = time_lw_Pprime.cpp \
			 randomize_serials.cpp timer.cpp timer.h
time_lw_Pprime_LDADD = ../ginac/libginac.la

time_lw_Q_SOURCES = time_lw_Q.cpp \
		    randomize_serials.cpp timer.cpp timer.h
time_lw_Q_LDADD = ../ginac/libginac.la

time_lw_Qprime_SOURCES = time_lw_Qprime.cpp \
			 randomize_serials.cpp timer.cpp timer.h
time_lw_Qprime_LDADD = ../ginac/libginac.la

time_antipode_SOURCES = time_antipode.cpp \
			randomize_serials.cpp timer.cpp timer.h
time_antipode_LDADD = ../ginac/libginac.la

time_fateman_expand_SOURCES = time_fateman_expand.cpp \
			      randomize_serials.cpp timer.cpp timer.h
time_fateman_expand_LDADD = ../ginac/libginac.la

time_uvar_gcd_SOURCES = time_uvar_gcd.cpp test_runner.h timer.cpp timer.h
time_uvar_gcd_LDADD = ../ginac/libginac.la

time_parser_SOURCES = time_parser.cpp \
		      randomize_serials.cpp timer.cpp timer.h
time_parser_LDADD = ../ginac/libginac.la

AM_CPPFLAGS = -I$(srcdir)/../ginac -I../ginac -DIN_GINAC

CLEANFILES = exam.gar
EXTRA_DIST = CMakeLists.txt
