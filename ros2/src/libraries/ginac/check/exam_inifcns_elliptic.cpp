/** @file exam_inifcns_nstdsums.cpp
 *
 *  This test routine applies assorted tests on initially known higher level
 *  functions. */

/*
 *  GiNaC Copyright (C) 1999-2023 Johannes Gutenberg University Mainz, Germany
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "ginac.h"
using namespace GiNaC;

#include <iostream>
#include <fstream>
using namespace std;


static unsigned check_q_expansion()
{
	unsigned err = 0;

	symbol q("q");
	int order = 200;

	ex res;

	// q-expansions from Sage up to order 200
	// notation Ek_N_|a|_|b|_K
	ex E1_12_1_4_1 = pow(q,2)+pow(q,4)+2*pow(q,5)+pow(q,8)+pow(q,9)+2*pow(q,10)+2*pow(q,13)+pow(q,16)+2*pow(q,17)+pow(q,18)+2*pow(q,20)+3*pow(q,25)+2*pow(q,26)
		+2*pow(q,29)+pow(q,32)+2*pow(q,34)+pow(q,36)+2*pow(q,37)+2*pow(q,40)+2*pow(q,41)+2*pow(q,45)+pow(q,49)+3*pow(q,50)+2*pow(q,52)+2*pow(q,53)+2*pow(q,58)
		+2*pow(q,61)+pow(q,64)+4*pow(q,65)+2*pow(q,68)+pow(q,72)+2*pow(q,73)+2*pow(q,74)+2*pow(q,80)+pow(q,81)+2*pow(q,82)+4*pow(q,85)+2*pow(q,89)+2*pow(q,90)
		+2*pow(q,97)+pow(q,98)+3*pow(q,100)+2*pow(q,101)+2*pow(q,104)+2*pow(q,106)+2*pow(q,109)+2*pow(q,113)+2*pow(q,116)+2*pow(q,117)+pow(q,121)+2*pow(q,122)
		+4*pow(q,125)+pow(q,128)+4*pow(q,130)+2*pow(q,136)+2*pow(q,137)+pow(q,144)+4*pow(q,145)+2*pow(q,146)+2*pow(q,148)+2*pow(q,149)+2*pow(q,153)+2*pow(q,157)
		+2*pow(q,160)+pow(q,162)+2*pow(q,164)+3*pow(q,169)+4*pow(q,170)+2*pow(q,173)+2*pow(q,178)+2*pow(q,180)+2*pow(q,181)+4*pow(q,185)+2*pow(q,193)
		+2*pow(q,194)+pow(q,196)+2*pow(q,197)+numeric(1,4)+q;

	ex E1_12_1_4_3 = numeric(1,4)+pow(q,3)+pow(q,6)+pow(q,12)+2*pow(q,15)+pow(q,24)+pow(q,27)+2*pow(q,30)+2*pow(q,39)+pow(q,48)+2*pow(q,51)+pow(q,54)+2*pow(q,60)
		+3*pow(q,75)+2*pow(q,78)+2*pow(q,87)+pow(q,96)+2*pow(q,102)+pow(q,108)+2*pow(q,111)+2*pow(q,120)+2*pow(q,123)+2*pow(q,135)+pow(q,147)+3*pow(q,150)
		+2*pow(q,156)+2*pow(q,159)+2*pow(q,174)+2*pow(q,183)+pow(q,192)+4*pow(q,195);

	ex E1_12_1_3_1 = 2*pow(q,175)+2*pow(q,189)+2*pow(q,199)+numeric(1,6)+2*pow(q,129)+4*pow(q,133)+2*pow(q,139)+2*pow(q,151)+2*pow(q,163)+2*pow(q,171)+2*pow(q,172)
		+2*pow(q,31)+2*pow(q,43)+2*pow(q,57)+2*pow(q,63)+2*pow(q,67)+2*pow(q,76)+2*pow(q,79)+2*pow(q,84)+4*pow(q,91)+2*pow(q,93)+2*pow(q,103)+2*pow(q,112)
		+2*pow(q,124)+2*pow(q,127)+pow(q,4)+pow(q,9)+2*pow(q,13)+pow(q,16)+pow(q,25)+pow(q,36)+2*pow(q,37)+3*pow(q,49)+2*pow(q,52)+2*pow(q,61)+pow(q,64)
		+2*pow(q,73)+pow(q,81)+2*pow(q,97)+pow(q,100)+2*pow(q,109)+2*pow(q,117)+pow(q,121)+pow(q,144)+2*pow(q,148)+2*pow(q,157)+3*pow(q,169)+2*pow(q,181)
		+2*pow(q,193)+3*pow(q,196)+pow(q,3)+pow(q,12)+pow(q,27)+2*pow(q,39)+pow(q,48)+pow(q,75)+pow(q,108)+2*pow(q,111)+3*pow(q,147)+2*pow(q,156)+2*pow(q,183)
		+pow(q,192)+2*pow(q,7)+2*pow(q,19)+2*pow(q,21)+2*pow(q,28)+q;

	ex E1_12_1_3_2 = numeric(1,6)+pow(q,2)+pow(q,8)+pow(q,18)+2*pow(q,26)+pow(q,32)+pow(q,50)+pow(q,72)+2*pow(q,74)+3*pow(q,98)+2*pow(q,104)+2*pow(q,122)+pow(q,128)
		+2*pow(q,146)+pow(q,162)+2*pow(q,194)+pow(q,6)+pow(q,24)+pow(q,54)+2*pow(q,78)+pow(q,96)+pow(q,150)+2*pow(q,14)+2*pow(q,38)+2*pow(q,42)+2*pow(q,56)
		+2*pow(q,62)+2*pow(q,86)+2*pow(q,114)+2*pow(q,126)+2*pow(q,134)+2*pow(q,152)+2*pow(q,158)+2*pow(q,168)+4*pow(q,182)+2*pow(q,186);

	ex E1_12_1_3_4 = numeric(1,6)+pow(q,4)+pow(q,12)+pow(q,16)+2*pow(q,28)+pow(q,36)+pow(q,48)+2*pow(q,52)+pow(q,64)+2*pow(q,76)+2*pow(q,84)+pow(q,100)+pow(q,108)
		+2*pow(q,112)+2*pow(q,124)+pow(q,144)+2*pow(q,148)+2*pow(q,156)+2*pow(q,172)+pow(q,192)+3*pow(q,196);


	ex E2_12_1_1_2 = 248*pow(q,175)+320*pow(q,189)+200*pow(q,199)+176*pow(q,129)+160*pow(q,133)+140*pow(q,139)+152*pow(q,151)+164*pow(q,163)+260*pow(q,171)
		+44*pow(q,172)+32*pow(q,31)+44*pow(q,43)+80*pow(q,57)+104*pow(q,63)+68*pow(q,67)+20*pow(q,76)+80*pow(q,79)+32*pow(q,84)+112*pow(q,91)+128*pow(q,93)
		+104*pow(q,103)+8*pow(q,112)+32*pow(q,124)+128*pow(q,127)+pow(q,2)+pow(q,4)+6*pow(q,5)+pow(q,8)+13*pow(q,9)+6*pow(q,10)+14*pow(q,13)+pow(q,16)
		+18*pow(q,17)+13*pow(q,18)+6*pow(q,20)+31*pow(q,25)+14*pow(q,26)+30*pow(q,29)+pow(q,32)+18*pow(q,34)+13*pow(q,36)+38*pow(q,37)+6*pow(q,40)+42*pow(q,41)
		+78*pow(q,45)+57*pow(q,49)+31*pow(q,50)+14*pow(q,52)+54*pow(q,53)+30*pow(q,58)+62*pow(q,61)+pow(q,64)+84*pow(q,65)+18*pow(q,68)+13*pow(q,72)+74*pow(q,73)
		+38*pow(q,74)+6*pow(q,80)+121*pow(q,81)+42*pow(q,82)+108*pow(q,85)+90*pow(q,89)+78*pow(q,90)+98*pow(q,97)+57*pow(q,98)+31*pow(q,100)+102*pow(q,101)
		+14*pow(q,104)+54*pow(q,106)+110*pow(q,109)+114*pow(q,113)+30*pow(q,116)+182*pow(q,117)+133*pow(q,121)+62*pow(q,122)+156*pow(q,125)+pow(q,128)+84*pow(q,130)
		+18*pow(q,136)+138*pow(q,137)+13*pow(q,144)+180*pow(q,145)+74*pow(q,146)+38*pow(q,148)+150*pow(q,149)+234*pow(q,153)+158*pow(q,157)+6*pow(q,160)+121*pow(q,162)
		+42*pow(q,164)+183*pow(q,169)+108*pow(q,170)+174*pow(q,173)+90*pow(q,178)+78*pow(q,180)+182*pow(q,181)+228*pow(q,185)+194*pow(q,193)+98*pow(q,194)+57*pow(q,196)
		+198*pow(q,197)+4*pow(q,3)+4*pow(q,6)+4*pow(q,12)+24*pow(q,15)+4*pow(q,24)+40*pow(q,27)+24*pow(q,30)+56*pow(q,39)+4*pow(q,48)+72*pow(q,51)+40*pow(q,54)
		+24*pow(q,60)+124*pow(q,75)+56*pow(q,78)+120*pow(q,87)+4*pow(q,96)+72*pow(q,102)+40*pow(q,108)+152*pow(q,111)+24*pow(q,120)+168*pow(q,123)+240*pow(q,135)
		+228*pow(q,147)+124*pow(q,150)+56*pow(q,156)+216*pow(q,159)+120*pow(q,174)+248*pow(q,183)+4*pow(q,192)+336*pow(q,195)+8*pow(q,7)+20*pow(q,19)+32*pow(q,21)
		+8*pow(q,28)+8*pow(q,14)+20*pow(q,38)+32*pow(q,42)+8*pow(q,56)+32*pow(q,62)+44*pow(q,86)+80*pow(q,114)+104*pow(q,126)+68*pow(q,134)+20*pow(q,152)+80*pow(q,158)
		+32*pow(q,168)+112*pow(q,182)+128*pow(q,186)+12*pow(q,11)+12*pow(q,22)+24*pow(q,23)+48*pow(q,33)+48*pow(q,35)+12*pow(q,44)+24*pow(q,46)+48*pow(q,47)+72*pow(q,55)
		+60*pow(q,59)+48*pow(q,66)+96*pow(q,69)+48*pow(q,70)+72*pow(q,71)+96*pow(q,77)+84*pow(q,83)+12*pow(q,88)+24*pow(q,92)+48*pow(q,94)+120*pow(q,95)+156*pow(q,99)
		+192*pow(q,105)+108*pow(q,107)+72*pow(q,110)+144*pow(q,115)+60*pow(q,118)+144*pow(q,119)+132*pow(q,131)+48*pow(q,132)+96*pow(q,138)+48*pow(q,140)+192*pow(q,141)
		+72*pow(q,142)+168*pow(q,143)+96*pow(q,154)+192*pow(q,155)+192*pow(q,161)+288*pow(q,165)+84*pow(q,166)+168*pow(q,167)+12*pow(q,176)+240*pow(q,177)+180*pow(q,179)
		+24*pow(q,184)+216*pow(q,187)+48*pow(q,188)+120*pow(q,190)+192*pow(q,191)+156*pow(q,198)+numeric(1,24)+q;

	ex E3_12_3_1_4 = 1850*pow(q,172)+362*pow(q,76)+450*pow(q,84)+650*pow(q,112)+962*pow(q,124)+pow(q,4)+3*pow(q,8)+13*pow(q,16)+24*pow(q,20)+51*pow(q,32)+81*pow(q,36)
		+72*pow(q,40)+170*pow(q,52)+205*pow(q,64)+288*pow(q,68)+243*pow(q,72)+312*pow(q,80)+601*pow(q,100)+510*pow(q,104)+840*pow(q,116)+819*pow(q,128)+864*pow(q,136)
		+1053*pow(q,144)+1370*pow(q,148)+1224*pow(q,160)+1680*pow(q,164)+1944*pow(q,180)+2451*pow(q,196)+9*pow(q,12)+27*pow(q,24)+117*pow(q,48)+216*pow(q,60)
		+459*pow(q,96)+729*pow(q,108)+648*pow(q,120)+1530*pow(q,156)+1845*pow(q,192)+50*pow(q,28)+150*pow(q,56)+1086*pow(q,152)+1350*pow(q,168)+120*pow(q,44)
		+360*pow(q,88)+528*pow(q,92)+1080*pow(q,132)+1200*pow(q,140)+1560*pow(q,176)+1584*pow(q,184)+2208*pow(q,188);

	// basis of Eisenstein space for Gamma_1(12) of weight 1
	res =  series_to_poly(Eisenstein_kernel(1, 12, 1, -3, 1).q_expansion_modular_form(q, order)) - E1_12_1_3_1;
	if ( res != 0 ) err++;

	res =  series_to_poly(Eisenstein_kernel(1, 12, 1, -3, 2).q_expansion_modular_form(q, order)) - E1_12_1_3_2;
	if ( res != 0 ) err++;

	res =  series_to_poly(Eisenstein_kernel(1, 12, 1, -3, 4).q_expansion_modular_form(q, order)) - E1_12_1_3_4;
	if ( res != 0 ) err++;

	res =  series_to_poly(Eisenstein_kernel(1, 12, 1, -4, 1).q_expansion_modular_form(q, order)) - E1_12_1_4_1;
	if ( res != 0 ) err++;

	res =  series_to_poly(Eisenstein_kernel(1, 12, 1, -4, 3).q_expansion_modular_form(q, order)) - E1_12_1_4_3;
	if ( res != 0 ) err++;

	// test one series of weight 2
	res =  series_to_poly(Eisenstein_kernel(2, 12, 1, 1, 2).q_expansion_modular_form(q, order)) - E2_12_1_1_2;
	if ( res != 0 ) err++;

	// and one of weight 3
	res =  series_to_poly(Eisenstein_kernel(3, 12, -3, 1, 4).q_expansion_modular_form(q, order)) - E3_12_3_1_4;
	if ( res != 0 ) err++;

	return err;
}

static unsigned check_polylogs()
{
	unsigned err = 0;

	int digitsbuf = Digits;
	Digits = 100;
	ex prec = 5 * pow(10, -(ex)Digits);

	ex y = numeric(9,10);

	ex z2 = numeric(2);
	ex z3 = numeric(3);

	ex L0 =  basic_log_kernel();
	ex omega_2 = multiple_polylog_kernel(z2);
	ex omega_3 = multiple_polylog_kernel(z3);

	ex res1,res2;

	res1 = G(lst{z2},y).evalf();
	res2 = iterated_integral(lst{omega_2},y).evalf();
	if ( abs(res1-res2) > prec ) err++;

	res1 = G(lst{0},y).evalf();
	res2 = iterated_integral(lst{L0},y).evalf();
	if ( abs(res1-res2) > prec ) err++;

	res1 = G(lst{z2,0},y).evalf();
	res2 = iterated_integral(lst{omega_2,L0},y).evalf();
	if ( abs(res1-res2) > prec ) err++;

	res1 = G(lst{0,0},y).evalf();
	res2 = iterated_integral(lst{L0,L0},y).evalf();
	if ( abs(res1-res2) > prec ) err++;

	res1 = G(lst{z2,0,0},y).evalf();
	res2 = iterated_integral(lst{omega_2,L0,L0},y).evalf();
	if ( abs(res1-res2) > prec ) err++;

	Digits = digitsbuf;

	return err;
}

static unsigned check_iterated_integral_modular_form_versus_Kronecker_dtau()
{
	unsigned err = 0;

	int digitsbuf = Digits;
	Digits = 30;
	ex prec = 5 * pow(10, -(ex)Digits);

	ex tau_6 = I;
	ex qbar_6 = exp(2*Pi*I*tau_6);
    	ex omega_0 = basic_log_kernel();

	ex eta_1   = Eisenstein_kernel(3, 6, -3, 1, 1);
	ex eta_2   = Eisenstein_kernel(3, 6, -3, 1, 2);
	ex omega_3 = modular_form_kernel(3, eta_1-8*eta_2);
	ex res1 = iterated_integral(lst{omega_0,omega_3},qbar_6).evalf();

	ex C_3  = I/sqrt(numeric(3));
	ex g3_1 = Kronecker_dtau_kernel(3,numeric(1,3),1,C_3);
	ex g3_2 = Kronecker_dtau_kernel(3,numeric(1,3),2,C_3);
    	ex expr = iterated_integral(lst{omega_0,g3_1},qbar_6) - 4*iterated_integral(lst{omega_0,g3_2},qbar_6);
	ex res2 = expr.evalf();

	if ( abs(res1-res2) > prec ) err++;

	Digits = digitsbuf;

	return err;
}

static unsigned check_modular_trafo()
{
	unsigned err = 0;

	int digitsbuf = Digits;
	Digits = 50;
	ex prec = 5 * pow(10, -(ex)Digits);

	int N_trunc = 100;

	int a = 0;
	int b = -1;
	int c = 1;
	int d = 0;

	ex tau = numeric(1,10)+numeric(4,5)*I;
	ex qbar = evalf(exp(2*Pi*I*tau));
	ex qbar_2 = evalf(exp(2*Pi*I*tau*numeric(1,2)));
	ex qbar_3 = evalf(exp(2*Pi*I*tau*numeric(1,3)));
	ex qbar_4 = evalf(exp(2*Pi*I*tau*numeric(1,4)));
	ex qbar_6 = evalf(exp(2*Pi*I*tau*numeric(1,6)));
	ex qbar_12 = evalf(exp(2*Pi*I*tau*numeric(1,12)));

	ex tau_prime = (a*tau+b)/(c*tau+d);
	ex qbar_prime = evalf(exp(2*Pi*I*tau_prime));
	ex qbar_prime_2 = evalf(exp(2*Pi*I*tau_prime*numeric(1,2)));
	ex qbar_prime_3 = evalf(exp(2*Pi*I*tau_prime*numeric(1,3)));
	ex qbar_prime_4 = evalf(exp(2*Pi*I*tau_prime*numeric(1,4)));
	ex qbar_prime_6 = evalf(exp(2*Pi*I*tau_prime*numeric(1,6)));
	ex qbar_prime_12 = evalf(exp(2*Pi*I*tau_prime*numeric(1,12)));

	numeric k,N,r,s;
	ex eta,eta_trafo,res1,res2;

	k = 4;
	N = 1;
	eta = Eisenstein_kernel(k, N, 1, 1, 1);
	res1 = ex_to<Eisenstein_kernel>(eta).get_numerical_value(qbar_prime,N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_kernel>(eta).get_numerical_value(qbar,N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 4;
	N = 2;
	r = 0;
	s = 0;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_2,N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_2,N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 4;
	N = 2;
	r = 1;
	s = 1;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_2,N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_2,N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 4;
	N = 4;
	r = 2;
	s = 2;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_4,N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_4,N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 4;
	N = 6;
	r = 3;
	s = 3;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_6,3*N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_6,3*N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 4;
	N = 12;
	r = 6;
	s = 6;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_12,6*N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_12,6*N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 4;
	N = 2;
	r = 1;
	s = 0;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_2,N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_2,N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 6;
	N = 6;
	r = 1;
	s = 5;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_6,2*N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_6,2*N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 6;
	N = 12;
	r = 2;
	s = 10;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_12,4*N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_12,4*N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 4;
	N = 3;
	r = 1;
	s = 2;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_3,N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_3,N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	k = 1;
	N = 6;
	r = 1;
	s = 5;
	eta       = Eisenstein_h_kernel(k, N, r, s);
	eta_trafo = Eisenstein_h_kernel(k, N, mod(r*d+s*b,N), mod(r*c+s*a,N));
	res1 = ex_to<Eisenstein_h_kernel>(eta).get_numerical_value(qbar_prime_6,2*N_trunc);
	res2 = pow(c*tau+d,k)*ex_to<Eisenstein_h_kernel>(eta_trafo).get_numerical_value(qbar_6,2*N_trunc);
	if ( abs(res1-res2) > prec ) err++;

	Digits = digitsbuf;

	return err;
}

static unsigned check_Kronecker_g()
{
	unsigned err = 0;

	int digitsbuf = Digits;
	Digits = 20;
	ex prec = 5 * pow(10, -(ex)Digits);

	ex tau = numeric(1,10)+numeric(2)*I;
	ex qbar = evalf(exp(2*Pi*I*tau));

	ex z = numeric(2,100)+numeric(1,10)*I;
	ex wbar = evalf(exp(2*Pi*I*z));

	ex z_j = numeric(-1,10)*I;

	ex res1,res2,res3;

	res1 = Kronecker_dtau_kernel(0,z).get_numerical_value(qbar);
	res2 = Kronecker_dz_kernel(1,0,tau).get_numerical_value(z);
	res3 = Kronecker_dz_kernel(1,z_j,tau).get_numerical_value(z+z_j);
	if ( abs(res1-res2) > prec ) err++;
	if ( abs(res1-res3) > prec ) err++;

	res1 = Kronecker_dtau_kernel(1,z).get_numerical_value(qbar);
	res2 = Kronecker_dz_kernel(2,0,tau).get_numerical_value(z);
	res3 = Kronecker_dz_kernel(2,z_j,tau).get_numerical_value(z+z_j);
	if ( abs(res1-res2) > prec ) err++;
	if ( abs(res1-res3) > prec ) err++;

	res1 = Kronecker_dtau_kernel(2,z).get_numerical_value(qbar);
	res2 = Kronecker_dz_kernel(3,0,tau).get_numerical_value(z);
	res3 = Kronecker_dz_kernel(3,z_j,tau).get_numerical_value(z+z_j);
	if ( abs(res1-res2) > prec ) err++;
	if ( abs(res1-res3) > prec ) err++;
    
	res1 = Kronecker_dtau_kernel(3,z).get_numerical_value(qbar);
	res2 = Kronecker_dz_kernel(4,0,tau).get_numerical_value(z);
	res3 = Kronecker_dz_kernel(4,z_j,tau).get_numerical_value(z+z_j);
	if ( abs(res1-res2) > prec ) err++;
	if ( abs(res1-res3) > prec ) err++;
    
	res1 = Kronecker_dtau_kernel(4,z).get_numerical_value(qbar);
	res2 = Kronecker_dz_kernel(5,0,tau).get_numerical_value(z);
	res3 = Kronecker_dz_kernel(5,z_j,tau).get_numerical_value(z+z_j);
	if ( abs(res1-res2) > prec ) err++;
	if ( abs(res1-res3) > prec ) err++;
    
	res1 = Kronecker_dtau_kernel(5,z).get_numerical_value(qbar);
	res2 = Kronecker_dz_kernel(6,0,tau).get_numerical_value(z);
	res3 = Kronecker_dz_kernel(6,z_j,tau).get_numerical_value(z+z_j);
	if ( abs(res1-res2) > prec ) err++;
	if ( abs(res1-res3) > prec ) err++;

	Digits = digitsbuf;

	return err;
}

unsigned exam_inifcns_elliptic(void)
{
	unsigned result = 0;
	
	cout << "examining consistency of iterated integrals" << flush;
	
	result += check_q_expansion();
	result += check_polylogs();
	result += check_iterated_integral_modular_form_versus_Kronecker_dtau();
	result += check_modular_trafo();
	result += check_Kronecker_g();
	
	return result;
}

int main(int argc, char** argv)
{
	return exam_inifcns_elliptic();
}
