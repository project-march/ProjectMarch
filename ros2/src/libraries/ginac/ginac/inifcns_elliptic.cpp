/** @file inifcns_elliptic.cpp
 *
 *  Implementation of some special functions related to elliptic curves
 *
 *  The functions are:
 *    complete elliptic integral of the first kind        EllipticK(k)
 *    complete elliptic integral of the second kind       EllipticE(k)
 *    iterated integral                                   iterated_integral(a,y) or iterated_integral(a,y,N_trunc)
 *
 *  Some remarks:
 *
 *    - All formulae used can be looked up in the following publication:
 *      [WW] Numerical evaluation of iterated integrals related to elliptic Feynman integrals, M.Walden, S.Weinzierl, arXiv:2010.05271
 *
 *    - When these routines and methods are used for scientific work that leads to publication in a scientific journal, 
 *      please refer to this program as : 
 *       M.Walden, S.Weinzierl, "Numerical evaluation of iterated integrals related to elliptic Feynman integrals", arXiv:2010.05271
 *
 *    - As these routines build on the core part of GiNaC, it is also polite to acknowledge
 *       C. Bauer, A. Frink, R. Kreckel, "Introduction to the GiNaC Framework for Symbolic Computation within the C++ Programming Language", 
 *       J. Symbolic Computations 33, 1 (2002), cs.sc/0004015
 *
 */

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

#include "inifcns.h"

#include "add.h"
#include "constant.h"
#include "lst.h"
#include "mul.h"
#include "numeric.h"
#include "operators.h"
#include "power.h"
#include "pseries.h"
#include "relational.h"
#include "symbol.h"
#include "utils.h"
#include "wildcard.h"

#include "integration_kernel.h"
#include "utils_multi_iterator.h"

#include <cln/cln.h>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cmath>

namespace GiNaC {


//////////////////////////////////////////////////////////////////////
//
// Complete elliptic integrals
//
// helper functions
//
//////////////////////////////////////////////////////////////////////


// anonymous namespace for helper function
namespace {

// Computes the arithmetic geometric of two numbers a_0 and b_0
cln::cl_N arithmetic_geometric_mean(const cln::cl_N & a_0, const cln::cl_N & b_0)
{
	cln::cl_N a_old = a_0 * cln::cl_float(1, cln::float_format(Digits));
	cln::cl_N b_old = b_0 * cln::cl_float(1, cln::float_format(Digits));
	cln::cl_N a_new;
	cln::cl_N b_new;
	cln::cl_N res = a_old;
	cln::cl_N resbuf;
	do {
		resbuf = res;

                a_new = (a_old+b_old)/2;
                b_new = sqrt(a_old*b_old);

		if ( ( abs(a_new-b_new) > abs(a_new+b_new) ) 
		     || 
		     ( (abs(a_new-b_new) == abs(a_new+b_new)) && (imagpart(b_new/a_new) <= 0) ) ) {
			b_new *= -1;
		}

		res = a_new;
		a_old = a_new;
		b_old = b_new;
    
	} while (res != resbuf);
	return res;
}

// Computes
//  a0^2 - sum_{n=0}^infinity 2^{n-1}*c_n^2
// with
//  c_{n+1} = c_n^2/4/a_{n+1}
//
// Needed for the complete elliptic integral of the second kind.
//
cln::cl_N agm_helper_second_kind(const cln::cl_N & a_0, const cln::cl_N & b_0, const cln::cl_N & c_0)
{
	cln::cl_N a_old = a_0 * cln::cl_float(1, cln::float_format(Digits));
	cln::cl_N b_old = b_0 * cln::cl_float(1, cln::float_format(Digits));
	cln::cl_N c_old = c_0 * cln::cl_float(1, cln::float_format(Digits));
	cln::cl_N a_new;
	cln::cl_N b_new;
	cln::cl_N c_new;
	cln::cl_N res = square(a_old)-square(c_old)/2;
	cln::cl_N resbuf;
	cln::cl_N pre = cln::cl_float(1, cln::float_format(Digits));
	do {
		resbuf = res;

                a_new = (a_old+b_old)/2;
                b_new = sqrt(a_old*b_old);

		if ( ( abs(a_new-b_new) > abs(a_new+b_new) ) 
		     || 
		     ( (abs(a_new-b_new) == abs(a_new+b_new)) && (imagpart(b_new/a_new) <= 0) ) ) {
			b_new *= -1;
		}

		c_new = square(c_old)/4/a_new;

		res -= pre*square(c_new);

		a_old = a_new;
		b_old = b_new;
		c_old = c_new;
		pre *= 2;
    
	} while (res != resbuf);
	return res;
}


} // end of anonymous namespace


//////////////////////////////////////////////////////////////////////
//
// Complete elliptic integrals
//
// GiNaC function
//
//////////////////////////////////////////////////////////////////////

static ex EllipticK_evalf(const ex& k)
{
	if ( !k.info(info_flags::numeric) ) {
		return EllipticK(k).hold();
	}
     
	cln::cl_N kbar = sqrt(1-square(ex_to<numeric>(k).to_cl_N()));

	ex result = Pi/2/numeric(arithmetic_geometric_mean(1,kbar));

	return result.evalf();
}


static ex EllipticK_eval(const ex& k)
{
	if (k == _ex0) {
		return Pi/2;
	}

	if ( k.info(info_flags::numeric) && !k.info(info_flags::crational) ) {
		return EllipticK(k).evalf();
	}

	return EllipticK(k).hold();
}


static ex EllipticK_deriv(const ex& k, unsigned deriv_param)
{
        return -EllipticK(k)/k + EllipticE(k)/k/(1-k*k);
}


static ex EllipticK_series(const ex& k, const relational& rel, int order, unsigned options)
{       
	const ex k_pt = k.subs(rel, subs_options::no_pattern);

	if (k_pt == _ex0) {
		const symbol s;
		ex ser;
		// manually construct the primitive expansion
		for (int i=0; i<(order+1)/2; ++i)
		{
			ser += Pi/2 * numeric(cln::square(cln::binomial(2*i,i))) * pow(s/4,2*i);
		}
		// substitute the argument's series expansion
		ser = ser.subs(s==k.series(rel, order), subs_options::no_pattern);
		// maybe that was terminating, so add a proper order term
		epvector nseq { expair(Order(_ex1), order) };
		ser += pseries(rel, std::move(nseq));
		// reexpanding it will collapse the series again
		return ser.series(rel, order);
	}

	if ( (k_pt == _ex1) || (k_pt == _ex_1) ) {
		throw std::runtime_error("EllipticK_series: don't know how to do the series expansion at this point!");
	}

	// all other cases
	throw do_taylor();
}

static void EllipticK_print_latex(const ex& k, const print_context& c)
{
	c.s << "\\mathrm{K}(";
	k.print(c);
	c.s << ")";
}


REGISTER_FUNCTION(EllipticK,
                  evalf_func(EllipticK_evalf).
                  eval_func(EllipticK_eval).
                  derivative_func(EllipticK_deriv).
                  series_func(EllipticK_series).
                  print_func<print_latex>(EllipticK_print_latex).
                  do_not_evalf_params());


static ex EllipticE_evalf(const ex& k)
{
	if ( !k.info(info_flags::numeric) ) {
		return EllipticE(k).hold();
	}

	cln::cl_N kbar = sqrt(1-square(ex_to<numeric>(k).to_cl_N()));

	ex result = Pi/2 * numeric( agm_helper_second_kind(1,kbar,ex_to<numeric>(k).to_cl_N()) / arithmetic_geometric_mean(1,kbar) );

	return result.evalf();
}


static ex EllipticE_eval(const ex& k)
{
	if (k == _ex0) {
		return Pi/2;
	}

	if ( (k == _ex1) || (k == _ex_1) ) {
		return 1;
	}

	if ( k.info(info_flags::numeric) && !k.info(info_flags::crational) ) {
		return EllipticE(k).evalf();
	}

	return EllipticE(k).hold();
}


static ex EllipticE_deriv(const ex& k, unsigned deriv_param)
{
        return -EllipticK(k)/k + EllipticE(k)/k;
}


static ex EllipticE_series(const ex& k, const relational& rel, int order, unsigned options)
{       
	const ex k_pt = k.subs(rel, subs_options::no_pattern);

	if (k_pt == _ex0) {
		const symbol s;
		ex ser;
		// manually construct the primitive expansion
		for (int i=0; i<(order+1)/2; ++i)
		{
			ser -= Pi/2 * numeric(cln::square(cln::binomial(2*i,i)))/(2*i-1) * pow(s/4,2*i);
		}
		// substitute the argument's series expansion
		ser = ser.subs(s==k.series(rel, order), subs_options::no_pattern);
		// maybe that was terminating, so add a proper order term
		epvector nseq { expair(Order(_ex1), order) };
		ser += pseries(rel, std::move(nseq));
		// reexpanding it will collapse the series again
		return ser.series(rel, order);
	}

	if ( (k_pt == _ex1) || (k_pt == _ex_1) ) {
		throw std::runtime_error("EllipticE_series: don't know how to do the series expansion at this point!");
	}

	// all other cases
	throw do_taylor();
}

static void EllipticE_print_latex(const ex& k, const print_context& c)
{
	c.s << "\\mathrm{K}(";
	k.print(c);
	c.s << ")";
}


REGISTER_FUNCTION(EllipticE,
                  evalf_func(EllipticE_evalf).
                  eval_func(EllipticE_eval).
                  derivative_func(EllipticE_deriv).
                  series_func(EllipticE_series).
                  print_func<print_latex>(EllipticE_print_latex).
                  do_not_evalf_params());


//////////////////////////////////////////////////////////////////////
//
// Iterated integrals
//
// helper functions
//
//////////////////////////////////////////////////////////////////////

// anonymous namespace for helper function
namespace {

// performs the actual series summation for an iterated integral
cln::cl_N iterated_integral_do_sum(const std::vector<int> & m, const std::vector<const integration_kernel *> & kernel, const cln::cl_N & lambda, int N_trunc)
{
        if ( cln::zerop(lambda) ) {
		return 0;
	}

	cln::cl_F one = cln::cl_float(1, cln::float_format(Digits));

	const int depth = m.size();

	cln::cl_N res = 0;
	cln::cl_N resbuf;
	cln::cl_N subexpr;

	if ( N_trunc == 0 ) {
		// sum until precision is reached
		bool flag_accidental_zero = false;

		int N = 1;

		do {
			resbuf = res;

			if ( depth > 1 ) {
				subexpr = 0;
				multi_iterator_ordered_eq<int> i_multi(1,N+1,depth-1);
				for( i_multi.init(); !i_multi.overflow(); i_multi++) {
					cln::cl_N tt = one;
					for (int j=1; j<depth; j++) {
						if ( j==1 ) {
							tt = tt * kernel[0]->series_coeff(N-i_multi[depth-2]) / cln::expt(cln::cl_I(i_multi[depth-2]),m[1]);
						}
						else {
							tt = tt * kernel[j-1]->series_coeff(i_multi[depth-j]-i_multi[depth-j-1]) / cln::expt(cln::cl_I(i_multi[depth-j-1]),m[j]);
						}
					}
					tt = tt * kernel[depth-1]->series_coeff(i_multi[0]);
					subexpr += tt;
				}
			}
			else {
				// depth == 1
				subexpr = kernel[0]->series_coeff(N) * one;
			}
			flag_accidental_zero = cln::zerop(subexpr);
			res += cln::expt(lambda, N) / cln::expt(cln::cl_I(N),m[0]) * subexpr;
			N++;

		} while ( (res != resbuf) || flag_accidental_zero );
	}
	else {
		// N_trunc > 0, sum up the first N_trunc terms
		for (int N=1; N<=N_trunc; N++) {
			if ( depth > 1 ) {
				subexpr = 0;
				multi_iterator_ordered_eq<int> i_multi(1,N+1,depth-1);
				for( i_multi.init(); !i_multi.overflow(); i_multi++) {
					cln::cl_N tt = one;
					for (int j=1; j<depth; j++) {
						if ( j==1 ) {
							tt = tt * kernel[0]->series_coeff(N-i_multi[depth-2]) / cln::expt(cln::cl_I(i_multi[depth-2]),m[1]);
						}
						else {
							tt = tt * kernel[j-1]->series_coeff(i_multi[depth-j]-i_multi[depth-j-1]) / cln::expt(cln::cl_I(i_multi[depth-j-1]),m[j]);
						}
					}
					tt = tt * kernel[depth-1]->series_coeff(i_multi[0]);
					subexpr += tt;
				}
			}
			else {
				// depth == 1
				subexpr = kernel[0]->series_coeff(N) * one;
			}
			res += cln::expt(lambda, N) / cln::expt(cln::cl_I(N),m[0]) * subexpr;
		}
	}

	return res;
}

// figure out the number of basic_log_kernels before a non-basic_log_kernel
cln::cl_N iterated_integral_prepare_m_lst(const std::vector<const integration_kernel *> & kernel_in, const cln::cl_N & lambda, int N_trunc)
{
	size_t depth = kernel_in.size();

	std::vector<int> m;
	m.reserve(depth);

	std::vector<const integration_kernel *> kernel;
	kernel.reserve(depth);

	int n = 1;

	for (const auto & it : kernel_in) {
		if ( is_a<basic_log_kernel>(*it) ) {
			n++;
		}
		else {
			m.push_back(n);
			kernel.push_back( &ex_to<integration_kernel>(*it) );
			n = 1;
		}
	}

	cln::cl_N result = iterated_integral_do_sum(m, kernel, lambda, N_trunc);

	return result;
}

// shuffle to remove trailing zeros,
// integration kernels, which are not basic_log_kernels, are treated as regularised kernels
cln::cl_N iterated_integral_shuffle(const std::vector<const integration_kernel *> & kernel, const cln::cl_N & lambda, int N_trunc)
{
        cln::cl_F one = cln::cl_float(1, cln::float_format(Digits));

	const size_t depth = kernel.size();

	size_t i_trailing = 0;
	for (size_t i=0; i<depth; i++) {
		if ( !(is_a<basic_log_kernel>(*(kernel[i]))) ) {
			i_trailing = i+1;
		}
	}

	if ( i_trailing == 0 ) {
		return cln::expt(cln::log(lambda), depth) / cln::factorial(depth) * one;
	}

	if ( i_trailing == depth ) {
		return iterated_integral_prepare_m_lst(kernel, lambda, N_trunc);
	}

	// shuffle
	std::vector<const integration_kernel *> a,b;
	for (size_t i=0; i<i_trailing; i++) {
		a.push_back(kernel[i]);
	}
	for (size_t i=i_trailing; i<depth; i++) {
		b.push_back(kernel[i]);
	}

	cln::cl_N result = iterated_integral_prepare_m_lst(a, lambda, N_trunc) * cln::expt(cln::log(lambda), depth-i_trailing) / cln::factorial(depth-i_trailing);
	multi_iterator_shuffle_prime<const integration_kernel *> i_shuffle(a,b);
	for( i_shuffle.init(); !i_shuffle.overflow(); i_shuffle++) {
		std::vector<const integration_kernel *> new_kernel;
		new_kernel.reserve(depth);
		for (size_t i=0; i<depth; i++) {
			new_kernel.push_back(i_shuffle[i]);
		}

		result -= iterated_integral_shuffle(new_kernel, lambda, N_trunc);
	}

	return result;
}

} // end of anonymous namespace

//////////////////////////////////////////////////////////////////////
//
// Iterated integrals
//
// GiNaC function
//
//////////////////////////////////////////////////////////////////////

static ex iterated_integral_evalf_impl(const ex& kernel_lst, const ex& lambda, const ex& N_trunc)
{
        // sanity check
	if ((!kernel_lst.info(info_flags::list)) || (!lambda.evalf().info(info_flags::numeric)) || (!N_trunc.info(info_flags::nonnegint))) {
		return iterated_integral(kernel_lst,lambda,N_trunc).hold();
	}

        lst k_lst = ex_to<lst>(kernel_lst);

	bool flag_not_numeric = false;
	for (const auto & it : k_lst) {
		if ( !is_a<integration_kernel>(it) ) {
			flag_not_numeric = true;
		}
	}
	if ( flag_not_numeric) {
		return iterated_integral(kernel_lst,lambda,N_trunc).hold();
	}

	for (const auto & it : k_lst) {
		if ( !(ex_to<integration_kernel>(it).is_numeric()) ) {
			flag_not_numeric = true;
		}
	}
	if ( flag_not_numeric) {
		return iterated_integral(kernel_lst,lambda,N_trunc).hold();
	}

	// now we know that iterated_integral gives a number

	int N_trunc_int = ex_to<numeric>(N_trunc).to_int();

	// check trailing zeros
	const size_t depth = k_lst.nops();

	std::vector<const integration_kernel *> kernel_vec;
	kernel_vec.reserve(depth);

	for (const auto & it : k_lst) {
		kernel_vec.push_back( &ex_to<integration_kernel>(it) );
	}

	size_t i_trailing = 0;
	for (size_t i=0; i<depth; i++) {
		if ( !(kernel_vec[i]->has_trailing_zero()) ) {
			i_trailing = i+1;
		}
	}

	// split integral into regularised integrals and trailing basic_log_kernels
	// non-basic_log_kernels are treated as regularised kernels in call to iterated_integral_shuffle
	cln::cl_F one = cln::cl_float(1, cln::float_format(Digits));
	cln::cl_N lambda_cln = ex_to<numeric>(lambda.evalf()).to_cl_N();
	basic_log_kernel L0 = basic_log_kernel();

	cln::cl_N result;
	if ( is_a<basic_log_kernel>(*(kernel_vec[depth-1])) ) {
		result = 0;
	}
	else {
		result = iterated_integral_shuffle(kernel_vec, lambda_cln, N_trunc_int);
	}

	cln::cl_N coeff = one;
	for (size_t i_plus=depth; i_plus>i_trailing; i_plus--) {
		coeff = coeff * kernel_vec[i_plus-1]->series_coeff(0);
		kernel_vec[i_plus-1] = &L0;
		if ( i_plus==i_trailing+1 ) {
			result += coeff * iterated_integral_shuffle(kernel_vec, lambda_cln, N_trunc_int);
		}
		else {
			if ( !(is_a<basic_log_kernel>(*(kernel_vec[i_plus-2]))) ) {
				result += coeff * iterated_integral_shuffle(kernel_vec, lambda_cln, N_trunc_int);
			}
		}
	}

	return numeric(result);
}

static ex iterated_integral2_evalf(const ex& kernel_lst, const ex& lambda)
{
	return iterated_integral_evalf_impl(kernel_lst,lambda,0);
}

static ex iterated_integral3_evalf(const ex& kernel_lst, const ex& lambda, const ex& N_trunc)
{
	return iterated_integral_evalf_impl(kernel_lst,lambda,N_trunc);
}

static ex iterated_integral2_eval(const ex& kernel_lst, const ex& lambda)
{
	if ( lambda.info(info_flags::numeric) && !lambda.info(info_flags::crational) ) {
		return iterated_integral(kernel_lst,lambda).evalf();
	}

	return iterated_integral(kernel_lst,lambda).hold();
}

static ex iterated_integral3_eval(const ex& kernel_lst, const ex& lambda, const ex& N_trunc)
{
	if ( lambda.info(info_flags::numeric) && !lambda.info(info_flags::crational) ) {
		return iterated_integral(kernel_lst,lambda,N_trunc).evalf();
	}

	return iterated_integral(kernel_lst,lambda,N_trunc).hold();
}

unsigned iterated_integral2_SERIAL::serial =
	function::register_new(function_options("iterated_integral", 2).
	                       eval_func(iterated_integral2_eval).
	                       evalf_func(iterated_integral2_evalf).
                               do_not_evalf_params().
	                       overloaded(2));

unsigned iterated_integral3_SERIAL::serial =
	function::register_new(function_options("iterated_integral", 3).
	                       eval_func(iterated_integral3_eval).
	                       evalf_func(iterated_integral3_evalf).
                               do_not_evalf_params().
	                       overloaded(2));

} // namespace GiNaC

