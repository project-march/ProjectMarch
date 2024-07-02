/** @file integration_kernel.cpp
 *
 *  Implementation of GiNaC's integration kernels for iterated integrals. */

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

#include "integration_kernel.h"
#include "add.h"
#include "mul.h"
#include "operators.h"
#include "power.h"
#include "relational.h"
#include "symbol.h"
#include "constant.h"
#include "numeric.h"
#include "function.h"
#include "pseries.h"
#include "utils.h"
#include "inifcns.h"

#include <iostream>
#include <stdexcept>
#include <cln/cln.h>


namespace GiNaC {

// anonymous namespace for helper function
namespace {

/**
 *
 * Returns the Kronecker symbol in the case where
 *  a: integer
 *  n: unit or prime number
 *
 * If n is an odd prime number, the routine returns the Legendre symbol.
 *
 * If n is a unit (e.g n equals 1 or -1) or if n is an even prime number (the only case is n=2)
 * the routine returns the special values defined below.
 *
 * Ref.: Toshitsune Miyake, Modular Forms, Chapter 3.1
 *
 */
numeric kronecker_symbol_prime(const numeric & a, const numeric & n)
{
	if ( n == 1 ) {
		return 1;
	}
	else if ( n == -1 ) {
		if ( a >= 0 ) {
			return 1;
		}
		else {
			return -1;
		}
	}
	else if ( n == 2 ) {
		if ( GiNaC::smod(a,8) == 1 ) {
			return 1;
		}
		else if ( GiNaC::smod(a,8) == -1 ) {
			return 1;
		}
		else if  ( GiNaC::smod(a,8) == 3 ) {
			return -1;
		}
		else if ( GiNaC::smod(a,8) == -3 ) {
			return -1;
		}
		else {
			return 0;
		}
	}

	// n is an odd prime number
	return GiNaC::smod( pow(a,(n-1)/2), n);
}

/**
 *
 * n:     positive integer
 *
 * a:     discriminant of a quadratic field, defines primitive character phi
 * b:     discriminant of a quadratic field, defines primitive character psi
 * L=|a|: conductor of primitive character phi
 * M=|b|: conductor of primitive character psi
 *
 * k:     modular weight
 *
 * This function computes
 * \f[
 *      \sum\limits_{d | n} \psi(d) \phi(n/d) d^{k-1}
 * \f]
 *
 * Ref.: Eq.(5.3.1), William A. Stein, Modular Forms, A computational Approach;
 *
 *       Eq.(32), arXiv:1704.08895
 *
 */
numeric divisor_function(const numeric & n, const numeric & a, const numeric & b, const numeric & k)
{
	ex res = 0;

	for (numeric i1=1; i1<=n; i1++) {
		if ( irem(n,i1) == 0 ) {
			numeric ratio = n/i1;
			res += primitive_dirichlet_character(ratio,a) * primitive_dirichlet_character(i1,b) * pow(i1,k-1);
		}
	}

	return ex_to<numeric>(res);
}

/**
 *
 * k:     modular weight
 *
 * a:     discriminant of a quadratic field, defines primitive character phi
 * b:     discriminant of a quadratic field, defines primitive character psi
 * L=|a|: conductor of primitive character phi
 * M=|b|: conductor of primitive character psi
 *
 * This function computes the constant term of the q-expansion of an Eisenstein series.
 *
 * The coefficient is given by the equation below eq.(5.3.1) in William A. Stein, Modular Forms, A computational Approach;
 *
 * or by eq.(33), arXiv:1704.08895
 *
 */
numeric coefficient_a0(const numeric & k, const numeric & a, const numeric & b)
{
	ex conductor = abs(a);

	numeric a0;
	if ( conductor == 1 ) {
		a0 = -numeric(1,2)/k*generalised_Bernoulli_number(k,b);
	}
	else {
		a0 = 0;
	}

	return a0;
}

/**
 *
 * k:     modular weight
 * q:     exp(2 Pi I tau/M)
 *
 * a:     discriminant of a quadratic field, defines primitive character phi
 * b:     discriminant of a quadratic field, defines primitive character psi
 * L=|a|: conductor of primitive character phi
 * M=|b|: conductor of primitive character psi
 *
 * N:     truncation order
 *
 * Returns the q-expansion of an Eisenstein to order N (the q^(N-1)-term is included, q^N is neglected).
 *
 * Ref.: Eq.(5.3.1), William A. Stein, Modular Forms, A computational Approach;
 *
 *       Eq.(32), arXiv:1704.08895
 *
 */
ex eisenstein_series(const numeric & k, const ex & q, const numeric & a, const numeric & b, const numeric & N)
{
	ex res = coefficient_a0(k,a,b);

	for (numeric i1=1; i1<N; i1++) {
		res += divisor_function(i1,a,b,k) * pow(q,i1);
	}

	return res;
}

/**
 *
 * Returns the q_N-expansion of the Eisenstein series E_{k,a,b}(K tau_N)
 *
 */
ex E_eisenstein_series(const ex & q, const numeric & k, const numeric & N_level, const numeric & a, const numeric & b, const numeric & K, const numeric & N_order)
{
	int N_order_int = N_order.to_int();

	ex res = eisenstein_series(k,pow(q,K),a,b,iquo(N_order,K));

	res += Order(pow(q,N_order_int));
	res = res.series(q,N_order_int);

	return res;
}

/**
 *
 * In weight 2 we have a special case for trivial Dirichlet characters:
 * Returns the q_N-expansion of the Eisenstein series E_{2,1,1}(tau_N) - K E_{2,1,1}(K tau_N). 
 *
 */
ex B_eisenstein_series(const ex & q, const numeric & N_level, const numeric & K, const numeric & N_order)
{
	int N_order_int = N_order.to_int();

	ex res = eisenstein_series(2,q,1,1,N_order) - K*eisenstein_series(2,pow(q,K),1,1,iquo(N_order,K));

	res += Order(pow(q,N_order_int));
	res = res.series(q,N_order_int);

	return res;
}

/**
 *
 * A helper function to expand polynomials in Eisenstein series.
 *
 */
struct subs_q_expansion : public map_function
{
	subs_q_expansion(const ex & arg_qbar, int arg_order) : qbar(arg_qbar), order(arg_order)
		{}

	ex operator()(const ex & e)
		{
			if ( is_a<Eisenstein_kernel>(e) || is_a<Eisenstein_h_kernel>(e) ) {
				return series_to_poly(e.series(qbar,order));
			}
			else {
				return e.map(*this);
			}
		}

	ex qbar;
	int order;
};

/**
 *
 * \f[
 *     Li_{-n}(x),  n>=0
 * \f]
 *
 * This is a rational function in x.
 *
 * To speed things up, we cache it.
 *
 */
class Li_negative
{
	// ctors
public:
	Li_negative();

	// non-virtual functions 
public:
	ex get_symbolic_value(int n, const ex & x_val);
	ex get_numerical_value(int n, const ex & x_val);

	// member variables :
protected:
	static std::vector<ex> cache_vec;
	static symbol x;
};


Li_negative::Li_negative() {}

ex Li_negative::get_symbolic_value(int n, const ex & x_val)
{
	int n_cache = cache_vec.size();

	if ( n >= n_cache ) {
		for (int j=n_cache; j<=n; j++) {
			ex f;
			if ( j == 0 ) {
				f = x/(1-x);
			}
			else {
				f = normal( x*diff(cache_vec[j-1],x));
			}
			cache_vec.push_back( f );
		}
	}

	return cache_vec[n].subs(x==x_val);
}

ex Li_negative::get_numerical_value(int n, const ex & x_val)
{
	symbol x_symb("x_symb");

	ex f = this->get_symbolic_value(n,x_symb);

	ex res = f.subs(x_symb==x_val).evalf();

	return res;
}

// initialise static data members
std::vector<ex> Li_negative::cache_vec;
symbol Li_negative::x = symbol("x");


} // end of anonymous namespace

/**
 *
 * Returns the decomposition of the positive integer n into prime numbers
 * in the form
 *  lst( lst(p1,...,pr), lst(a1,...,ar) )
 * such that
 *  n = p1^a1 ... pr^ar.
 *
 */
ex ifactor(const numeric & n)
{
	if ( !n.is_pos_integer() ) throw (std::runtime_error("ifactor(): argument not a positive integer"));

	lst p_lst, exp_lst;

	// implementation for small integers
	numeric n_temp = n;
	for (numeric p=2; p<=n; p++) {
		if ( p.info(info_flags::prime) ) {
			numeric exp_temp = 0;
			while ( irem(n_temp, p) == 0 ) {
				n_temp = n_temp/p;
				exp_temp++;
			}
			if ( exp_temp>0 ) {
				p_lst.append(p);
				exp_lst.append(exp_temp);
			}
		}
		if ( n_temp == 1 ) break;
	}

	if ( n_temp != 1 ) throw (std::runtime_error("ifactor(): probabilistic primality test failed"));

	lst res = {p_lst,exp_lst};

	return res;
}

/**
 *
 * Returns true if the integer n is either one or the discriminant of a quadratic number field.
 *
 * Returns false otherwise.
 *
 * Ref.: Toshitsune Miyake, Modular Forms, Chapter 3.1
 *
 */
bool is_discriminant_of_quadratic_number_field(const numeric & n)
{
	if ( n == 0 ) {
		return false;
	}

	if ( n == 1 ) {
		return true;
	}

	lst prime_factorisation = ex_to<lst>(ifactor(abs(n)));
	lst p_lst = ex_to<lst>(prime_factorisation.op(0));
	lst e_lst = ex_to<lst>(prime_factorisation.op(1));

	size_t n_primes = p_lst.nops();

	if ( n_primes > 0 ) {
		// take the last prime
		numeric p = ex_to<numeric>(p_lst.op(n_primes-1));
	
		if ( p.is_odd() ) {
			if ( e_lst.op(n_primes-1) != 1 ) {
				return false;
			}

			numeric pstar = p;
			if ( mod(p,4) == 3 ) {
				pstar = -p;
			}
			return is_discriminant_of_quadratic_number_field(n/pstar);
		}
	}
	// power of two now
	if ( (n==-4) || (n==-8) || (n==8) || (n==-32) || (n==32) || (n==-64) || (n==128) ) {
		return true; 
	}

	return false;
}

/**
 *
 * Returns the Kronecker symbol
 *  a: integer
 *  n: integer
 *
 * This routine defines
 *  kronecker_symbol(1,0)   = 1
 *  kronecker_symbol(-1,0)  = 1
 *  kronecker_symbol(a,0)   = 0, a != 1,-1
 *
 * In particular
 *  kronecker_symbol(-1,0) = 1 (in agreement with Sage)
 *
 * Ref.: Toshitsune Miyake, Modular Forms, Chapter 3.1
 *
 */
numeric kronecker_symbol(const numeric & a, const numeric & n)
{
	// case n=0 first, include kronecker_symbol(0,0)=0
	if ( n == 0 ) {
		if ( (a == 1) || (a == -1) ) {
			return 1;
		}
		else {
			return 0;
		}
	}

	numeric unit = 1;
	numeric n_pos = n;
	if ( n_pos<0 ) {
		unit = -1;
		n_pos = -n;
	}

	ex res = kronecker_symbol_prime(a,unit);

	numeric n_odd = n_pos;
	numeric alpha = 0;
	while ( n_odd.is_even() ) {
		alpha++;
		n_odd = n_odd/2;
	}
	if ( alpha>0 ) {
		res *= pow(kronecker_symbol_prime(a,2),alpha);
	}

	lst temp_lst = ex_to<lst>(ifactor(n_odd));
	lst prime_lst = ex_to<lst>(temp_lst.op(0));
	lst expo_lst = ex_to<lst>(temp_lst.op(1));

	for (auto it_p = prime_lst.begin(), it_e = expo_lst.begin(); it_p != prime_lst.end(); it_p++, it_e++) {
		res *= pow(kronecker_symbol_prime(a,ex_to<numeric>(*it_p)),ex_to<numeric>(*it_e));
	}

	return ex_to<numeric>(res);
}

/**
 *
 * Defines a primitive Dirichlet character through the Kronecker symbol.
 *
 *  n:  integer
 *  a:  discriminant of a quadratic field
 * |a|: conductor
 *
 * The character takes the values -1,0,1.
 *
 */
numeric primitive_dirichlet_character(const numeric & n, const numeric & a)
{
	return kronecker_symbol(a,n);
}

/**
 *
 * Defines a Dirichlet character through the Kronecker symbol.
 *
 *  n:  integer
 *  a:  discriminant of a quadratic field
 * |a|: conductor
 *  N:  modulus, needs to be multiple of |a|
 *
 * The character takes the values -1,0,1.
 *
 */
numeric dirichlet_character(const numeric & n, const numeric & a, const numeric & N)
{
	if ( gcd(n,N) == 1 ) {
		return primitive_dirichlet_character(n,a);
	}

	return 0;
}

/**
 *
 * The generalised Bernoulli number.
 *
 * k:     index / modular weight
 *
 * b:     discriminant of a quadratic field, defines primitive character psi
 * M=|b|: conductor of primitive character psi
 *
 * The generalised Bernoulli number is computed from the series expansion of the generating function.
 * The generating function is given in eq.(34), arXiv:1704.08895
 *
 */
numeric generalised_Bernoulli_number(const numeric & k, const numeric & b)
{
	int k_int = k.to_int();
 
	symbol x("x");

	numeric conductor = abs(b);

	ex gen_fct = 0;
	for (numeric i1=1; i1<=conductor; i1++) {
		gen_fct += primitive_dirichlet_character(i1,b) * x*exp(i1*x)/(exp(conductor*x)-1);
	}

	gen_fct = series_to_poly(gen_fct.series(x,k_int+1));
 
	ex B = factorial(k) * gen_fct.coeff(x,k_int);

	return ex_to<numeric>(B);
}

/**
 *
 * The Bernoulli polynomials
 *
 */
ex Bernoulli_polynomial(const numeric & k, const ex & x)
{
	int k_int = k.to_int();
 
	symbol t("t");

	ex gen_fct = t*exp(x*t)/(exp(t)-1);

	gen_fct = series_to_poly(gen_fct.series(t,k_int+1));
 
	ex B = factorial(k) * gen_fct.coeff(t,k_int);

	return B;
}



GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(integration_kernel, basic,
  print_func<print_context>(&integration_kernel::do_print))

integration_kernel::integration_kernel() : inherited(), cache_step_size(100), series_vec()
{
}

int integration_kernel::compare_same_type(const basic &other) const
{
	return 0;
}

ex integration_kernel::series(const relational & r, int order, unsigned options) const
{
	if ( r.rhs() != 0 ) {
		throw (std::runtime_error("integration_kernel::series: non-zero expansion point not implemented"));
	}

	return Laurent_series(r.lhs(),order);
}

/**
 *
 * This routine returns true, if the integration kernel has a trailing zero.
 *
 */
bool integration_kernel::has_trailing_zero(void) const
{
	if ( cln::zerop( series_coeff(0) ) ) {
		return false;
	}

	return true;
}

/**
 *
 * This routine returns true, if the integration kernel can be evaluated numerically.
 *
 */
bool integration_kernel::is_numeric(void) const
{
	return true;
}

/**
 *
 * Subclasses have either to implement series_coeff_impl
 * or the two methods Laurent_series and uses_Laurent_series.
 *
 * The method series_coeff_impl can be used, if a single coefficient can be computed 
 * independently of the others.
 *
 * The method Laurent_series can be used, if it is more efficient to compute a Laurent series
 * in one shot and to determine a range of coefficients from this Laurent series.
 *
 */
cln::cl_N integration_kernel::series_coeff(int i) const
{
	int n_vec = series_vec.size();

	if ( i >= n_vec ) {
		int N = cache_step_size*(i/cache_step_size+1);

		if ( uses_Laurent_series() ) {
			symbol x("x");
			// series_vec[0] gives coefficient of 1/z, series_vec[N-1] coefficient of z^(N-2),
			// thus expansion up to order (N-1) is required
			ex temp = Laurent_series(x, N-1);
			for (int j=n_vec; j<N; j++) {
				series_vec.push_back( ex_to<numeric>(temp.coeff(x,j-1).evalf()).to_cl_N() );
			}
		}
		else {
			for (int j=n_vec; j<N; j++) {
				series_vec.push_back( series_coeff_impl(j) );
			}
		}
	}

	return series_vec[i];
}

/**
 *
 * For \f$ \omega = d\lambda \f$ only the coefficient of \f$ \lambda^0 \f$ is non-zero.
 *
 * The i-th coefficient corresponds to the power \f$ \lambda^{i-1} \f$.
 *
 */
cln::cl_N integration_kernel::series_coeff_impl(int i) const
{
	if ( i == 1 ) {
		return 1;
	}

	return 0;
}

/**
 *
 * Returns the Laurent series, starting possibly with the pole term.
 * Neglected terms are of order \f$ O(x^order) \f$.
 *
 */
ex integration_kernel::Laurent_series(const ex & x, int order) const
{
	ex res = 0;
	for (int n=-1; n<order; n++) {
		res += numeric(series_coeff(n+1)) * pow(x,n);
	}
	res += Order(pow(x,order));
	res = res.series(x,order);

	return res;
}

/**
 *
 * Evaluates the integrand at lambda.
 *
 */
ex  integration_kernel::get_numerical_value(const ex & lambda, int N_trunc) const
{
	return get_numerical_value_impl(lambda, 1, 0, N_trunc);
}

/**
 *
 * Returns true, if the coefficients are computed from the Laurent series
 * (in which case the method Laurent_series needs to be implemented).
 *
 * Returns false if this is not the case 
 * (and the class has an implementation of series_coeff_impl).
 * 
 */
bool integration_kernel::uses_Laurent_series() const
{
	return false;
}

/**
 *
 * Returns the current size of the cache.
 *
 */
size_t integration_kernel::get_cache_size(void) const
{
	return series_vec.size();
}

/**
 *
 * Sets the step size by which the cache is increased.
 *
 */
void integration_kernel::set_cache_step(int cache_steps) const
{
	cache_step_size = cache_steps;
}

/**
 *
 * Wrapper around series_coeff(i), converts cl_N to numeric.
 *
 */
ex integration_kernel::get_series_coeff(int i) const
{
	return numeric(series_coeff(i));
}

/**
 *
 * The actual implementation for computing a numerical value for the integrand.
 *
 */
ex  integration_kernel::get_numerical_value_impl(const ex & lambda, const ex & pre, int shift, int N_trunc) const
{
	cln::cl_N lambda_cln = ex_to<numeric>(lambda.evalf()).to_cl_N();
	cln::cl_N pre_cln = ex_to<numeric>(pre.evalf()).to_cl_N();

	cln::cl_F one = cln::cl_float(1, cln::float_format(Digits));

	cln::cl_N res = 0;
	cln::cl_N resbuf;
	cln::cl_N subexpr;

	if ( N_trunc == 0 ) {
		// sum until precision is reached
		bool flag_accidental_zero = false;

		int N = 0;

		do {
			resbuf = res;
	 
			subexpr = series_coeff(N);

			res += pre_cln * subexpr * cln::expt(lambda_cln,N-1+shift);

			flag_accidental_zero = cln::zerop(subexpr);

			N++;
		} while ( (res != resbuf) || flag_accidental_zero );
	}
	else {
		// N_trunc > 0, sum up the first N_trunc terms
		for (int N=0; N<N_trunc; N++) {
			subexpr = series_coeff(N);

			res += pre_cln * subexpr * cln::expt(lambda_cln,N-1+shift);
		}
	}

	return numeric(res);
}

void integration_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "integration_kernel()";
}

GINAC_BIND_UNARCHIVER(integration_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(basic_log_kernel, integration_kernel,
				     print_func<print_context>(&basic_log_kernel::do_print))

basic_log_kernel::basic_log_kernel() : inherited()
{ 
}

int basic_log_kernel::compare_same_type(const basic &other) const
{
	return 0;
}

cln::cl_N basic_log_kernel::series_coeff_impl(int i) const
{
	if ( i == 0 ) {
		return 1;
	}

	return 0;
}

void basic_log_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "basic_log_kernel()";
}

GINAC_BIND_UNARCHIVER(basic_log_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(multiple_polylog_kernel, integration_kernel,
				     print_func<print_context>(&multiple_polylog_kernel::do_print))

multiple_polylog_kernel::multiple_polylog_kernel() : inherited(), z(_ex1)
{ 
}

multiple_polylog_kernel::multiple_polylog_kernel(const ex & arg_z) : inherited(), z(arg_z)
{
}

int multiple_polylog_kernel::compare_same_type(const basic &other) const
{
	const multiple_polylog_kernel &o = static_cast<const multiple_polylog_kernel &>(other);

	return z.compare(o.z);
}

size_t multiple_polylog_kernel::nops() const
{
	return 1;
}

ex multiple_polylog_kernel::op(size_t i) const
{
	if ( i != 0 ) {
		throw(std::range_error("multiple_polylog_kernel::op(): out of range"));
	}

	return z;
}

ex & multiple_polylog_kernel::let_op(size_t i)
{
	ensure_if_modifiable();

	if ( i != 0 ) {
		throw(std::range_error("multiple_polylog_kernel::let_op(): out of range"));
	}

	return z;
}

bool multiple_polylog_kernel::is_numeric(void) const
{
	return z.evalf().info(info_flags::numeric);
}

cln::cl_N multiple_polylog_kernel::series_coeff_impl(int i) const
{
	if ( i == 0 ) {
		return 0;
	}

	return -cln::expt(ex_to<numeric>(z.evalf()).to_cl_N(),-i);
}

void multiple_polylog_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "multiple_polylog_kernel(";
	z.print(c);
	c.s << ")";
}

GINAC_BIND_UNARCHIVER(multiple_polylog_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(ELi_kernel, integration_kernel,
				     print_func<print_context>(&ELi_kernel::do_print))

ELi_kernel::ELi_kernel() : inherited(), n(_ex0), m(_ex0), x(_ex0), y(_ex0)
{ 
}

ELi_kernel::ELi_kernel(const ex & arg_n, const ex & arg_m, const ex & arg_x, const ex & arg_y) : inherited(), n(arg_n), m(arg_m), x(arg_x), y(arg_y)
{
}

int ELi_kernel::compare_same_type(const basic &other) const
{
	const ELi_kernel &o = static_cast<const ELi_kernel &>(other);
	int cmpval;

	cmpval = n.compare(o.n);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = m.compare(o.m);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = x.compare(o.x);
	if ( cmpval) {
		return cmpval;
	}

	return y.compare(o.y);
}

size_t ELi_kernel::nops() const
{
	return 4;
}

ex ELi_kernel::op(size_t i) const
{
	switch (i) {
	case 0:
		return n;
	case 1:
		return m;
	case 2:
		return x;
	case 3:
		return y;
	default:
		throw (std::out_of_range("ELi_kernel::op() out of range"));
	}
}

ex & ELi_kernel::let_op(size_t i)
{
	ensure_if_modifiable();

	switch (i) {
	case 0:
		return n;
	case 1:
		return m;
	case 2:
		return x;
	case 3:
		return y;
	default:
		throw (std::out_of_range("ELi_kernel::let_op() out of range"));
	}
}

bool ELi_kernel::is_numeric(void) const
{
	return (n.info(info_flags::nonnegint) && m.info(info_flags::numeric) && x.evalf().info(info_flags::numeric) && y.evalf().info(info_flags::numeric));
}

cln::cl_N ELi_kernel::series_coeff_impl(int i) const
{
	if ( i == 0 ) {
		return 0;
	}

	int n_int = ex_to<numeric>(n).to_int();
	int m_int = ex_to<numeric>(m).to_int();

	cln::cl_N x_cln = ex_to<numeric>(x.evalf()).to_cl_N();
	cln::cl_N y_cln = ex_to<numeric>(y.evalf()).to_cl_N();

	cln::cl_N res_cln = 0;

	for (int j=1; j<=i; j++) {
		if ( (i % j) == 0 ) {
			int k = i/j;

			res_cln += cln::expt(x_cln,j)/cln::expt(cln::cl_I(j),n_int) * cln::expt(y_cln,k)/cln::expt(cln::cl_I(k),m_int);
		}
	}

	return res_cln;
}

/**
 *
 * Returns the value of ELi_{n,m}(x,y,qbar)
 *
 */
ex  ELi_kernel::get_numerical_value(const ex & qbar, int N_trunc) const
{
	return get_numerical_value_impl(qbar, 1, 1, N_trunc);
}

void ELi_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "ELi_kernel(";
	n.print(c);
	c.s << ",";
	m.print(c);
	c.s << ",";
	x.print(c);
	c.s << ",";
	y.print(c);
	c.s << ")";
}

GINAC_BIND_UNARCHIVER(ELi_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(Ebar_kernel, integration_kernel,
				     print_func<print_context>(&Ebar_kernel::do_print))

Ebar_kernel::Ebar_kernel() : inherited(), n(_ex0), m(_ex0), x(_ex0), y(_ex0)
{ 
}

Ebar_kernel::Ebar_kernel(const ex & arg_n, const ex & arg_m, const ex & arg_x, const ex & arg_y) : inherited(), n(arg_n), m(arg_m), x(arg_x), y(arg_y)
{
}

int Ebar_kernel::compare_same_type(const basic &other) const
{
	const Ebar_kernel &o = static_cast<const Ebar_kernel &>(other);
	int cmpval;

	cmpval = n.compare(o.n);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = m.compare(o.m);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = x.compare(o.x);
	if ( cmpval) {
		return cmpval;
	}

	return y.compare(o.y);
}

size_t Ebar_kernel::nops() const
{
	return 4;
}

ex Ebar_kernel::op(size_t i) const
{
	switch (i) {
	case 0:
		return n;
	case 1:
		return m;
	case 2:
		return x;
	case 3:
		return y;
	default:
		throw (std::out_of_range("Ebar_kernel::op() out of range"));
	}
}

ex & Ebar_kernel::let_op(size_t i)
{
	ensure_if_modifiable();

	switch (i) {
	case 0:
		return n;
	case 1:
		return m;
	case 2:
		return x;
	case 3:
		return y;
	default:
		throw (std::out_of_range("Ebar_kernel::let_op() out of range"));
	}
}

bool Ebar_kernel::is_numeric(void) const
{
	return (n.info(info_flags::nonnegint) && m.info(info_flags::numeric) && x.evalf().info(info_flags::numeric) && y.evalf().info(info_flags::numeric));
}

cln::cl_N Ebar_kernel::series_coeff_impl(int i) const
{
	if ( i == 0 ) {
		return 0;
	}

	int n_int = ex_to<numeric>(n).to_int();
	int m_int = ex_to<numeric>(m).to_int();

	cln::cl_N x_cln = ex_to<numeric>(x.evalf()).to_cl_N();
	cln::cl_N y_cln = ex_to<numeric>(y.evalf()).to_cl_N();

	cln::cl_N res_cln = 0;

	for (int j=1; j<=i; j++) {
		if ( (i % j) == 0 ) {
			int k = i/j;

			res_cln += (cln::expt(x_cln,j)*cln::expt(y_cln,k)-cln::expt(cln::cl_I(-1),n_int+m_int)*cln::expt(x_cln,-j)*cln::expt(y_cln,-k))/cln::expt(cln::cl_I(j),n_int)/cln::expt(cln::cl_I(k),m_int);
		}
	}

	return res_cln;
}

/**
 *
 * Returns the value of Ebar_{n,m}(x,y,qbar)
 *
 */
ex  Ebar_kernel::get_numerical_value(const ex & qbar, int N_trunc) const
{
	return get_numerical_value_impl(qbar, 1, 1, N_trunc);
}

void Ebar_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "Ebar_kernel(";
	n.print(c);
	c.s << ",";
	m.print(c);
	c.s << ",";
	x.print(c);
	c.s << ",";
	y.print(c);
	c.s << ")";
}

GINAC_BIND_UNARCHIVER(Ebar_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(Kronecker_dtau_kernel, integration_kernel,
				     print_func<print_context>(&Kronecker_dtau_kernel::do_print))

Kronecker_dtau_kernel::Kronecker_dtau_kernel() : inherited(), n(_ex0), z(_ex0), K(_ex1), C_norm(_ex1)
{ 
}

Kronecker_dtau_kernel::Kronecker_dtau_kernel(const ex & arg_n, const ex & arg_z, const ex & arg_K, const ex & arg_C_norm) : inherited(), n(arg_n), z(arg_z), K(arg_K), C_norm(arg_C_norm)
{
}

int Kronecker_dtau_kernel::compare_same_type(const basic &other) const
{
	const Kronecker_dtau_kernel &o = static_cast<const Kronecker_dtau_kernel &>(other);
	int cmpval;

	cmpval = n.compare(o.n);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = z.compare(o.z);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = K.compare(o.K);
	if ( cmpval) {
		return cmpval;
	}

	return C_norm.compare(o.C_norm);
}

size_t Kronecker_dtau_kernel::nops() const
{
	return 4;
}

ex Kronecker_dtau_kernel::op(size_t i) const
{
	switch (i) {
	case 0:
		return n;
	case 1:
		return z;
	case 2:
		return K;
	case 3:
		return C_norm;
	default:
		throw (std::out_of_range("Kronecker_dtau_kernel::op() out of range"));
	}
}

ex & Kronecker_dtau_kernel::let_op(size_t i)
{
	ensure_if_modifiable();

	switch (i) {
	case 0:
		return n;
	case 1:
		return z;
	case 2:
		return K;
	case 3:
		return C_norm;
	default:
		throw (std::out_of_range("Kronecker_dtau_kernel::let_op() out of range"));
	}
}

bool Kronecker_dtau_kernel::is_numeric(void) const
{
	return (n.info(info_flags::nonnegint) && z.evalf().info(info_flags::numeric) && K.info(info_flags::posint) && C_norm.evalf().info(info_flags::numeric));
}

cln::cl_N Kronecker_dtau_kernel::series_coeff_impl(int i) const
{
	numeric n_num = ex_to<numeric>(n);
	int n_int = n_num.to_int();

	// case n=0
	if ( n_num == 0 ) {
		if ( i == 0 ) {
			ex res = -C_norm*K;

			return ex_to<numeric>(res.evalf()).to_cl_N();
		}

		return 0;
	}

	// case n=1
	if ( n_num == 1 ) {
		return 0;
	}

	// case n>1
	if ( i == 0 ) {
		ex res = C_norm*K / factorial(n_num-2) * bernoulli(n_num)/n_num;

		return ex_to<numeric>(res.evalf()).to_cl_N();
	}

	// n>1, i>0

	// if K>1 the variable i needs to be a multiple of K
	int K_int = ex_to<numeric>(K).to_int();

	if ( (i % K_int) != 0 ) {
		return 0;
	}
	int i_local = i/K_int;

	ex w = exp(ex_to<numeric>((2*Pi*I*z).evalf()));
	cln::cl_N w_cln = ex_to<numeric>(w).to_cl_N();
	cln::cl_N res_cln = 0;
	for (int j=1; j<=i_local; j++) {
		if ( (i_local % j) == 0 ) {
			res_cln += (cln::expt(w_cln,j)+cln::expt(cln::cl_I(-1),n_int)*cln::expt(w_cln,-j)) * cln::expt(cln::cl_I(i_local/j),n_int-1); 
		}
	}
	ex pre = -C_norm*K/factorial(n_num-2);

	return ex_to<numeric>(pre.evalf()).to_cl_N() * res_cln;
}

/**
 *
 * Returns the value of the g^(n)(z,K*tau), where tau is given by qbar.
 *
 */
ex  Kronecker_dtau_kernel::get_numerical_value(const ex & qbar, int N_trunc) const
{
	numeric n_num = ex_to<numeric>(n);

	if ( n_num == 0 ) {
		return 1;
	}

	// use the direct formula here
	if ( n_num == 1 ) {
		ex wbar = exp(ex_to<numeric>((2*Pi*I*z).evalf()));
		ex res = -2*Pi*I*( numeric(1,2)*(1+wbar)/(1-wbar) + Ebar_kernel(0,0,wbar,1).get_numerical_value(pow(qbar,K),N_trunc));

		return ex_to<numeric>(res.evalf());
	}

	ex pre = pow(2*Pi*I,n_num)/C_norm/K/(n_num-1);

	return get_numerical_value_impl(qbar, pre, 1, N_trunc);
}

void Kronecker_dtau_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "Kronecker_dtau_kernel(";
	n.print(c);
	c.s << ",";
	z.print(c);
	c.s << ",";
	K.print(c);
	c.s << ",";
	C_norm.print(c);
	c.s << ")";
}

GINAC_BIND_UNARCHIVER(Kronecker_dtau_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(Kronecker_dz_kernel, integration_kernel,
				     print_func<print_context>(&Kronecker_dz_kernel::do_print))

Kronecker_dz_kernel::Kronecker_dz_kernel() : inherited(), n(_ex0), z_j(_ex0), tau(_ex0), K(_ex1), C_norm(_ex1)
{ 
}

Kronecker_dz_kernel::Kronecker_dz_kernel(const ex & arg_n, const ex & arg_z_j, const ex & arg_tau, const ex & arg_K, const ex & arg_C_norm) : inherited(), n(arg_n), z_j(arg_z_j), tau(arg_tau), K(arg_K), C_norm(arg_C_norm)
{
}

int Kronecker_dz_kernel::compare_same_type(const basic &other) const
{
	const Kronecker_dz_kernel &o = static_cast<const Kronecker_dz_kernel &>(other);
	int cmpval;

	cmpval = n.compare(o.n);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = z_j.compare(o.z_j);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = tau.compare(o.tau);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = K.compare(o.K);
	if ( cmpval) {
		return cmpval;
	}

	return C_norm.compare(o.C_norm);
}

size_t Kronecker_dz_kernel::nops() const
{
	return 5;
}

ex Kronecker_dz_kernel::op(size_t i) const
{
	switch (i) {
	case 0:
		return n;
	case 1:
		return z_j;
	case 2:
		return tau;
	case 3:
		return K;
	case 4:
		return C_norm;
	default:
		throw (std::out_of_range("Kronecker_dz_kernel::op() out of range"));
	}
}

ex & Kronecker_dz_kernel::let_op(size_t i)
{
	ensure_if_modifiable();

	switch (i) {
	case 0:
		return n;
	case 1:
		return z_j;
	case 2:
		return tau;
	case 3:
		return K;
	case 4:
		return C_norm;
	default:
		throw (std::out_of_range("Kronecker_dz_kernel::let_op() out of range"));
	}
}

bool Kronecker_dz_kernel::is_numeric(void) const
{
	return (n.info(info_flags::nonnegint) && z_j.evalf().info(info_flags::numeric) && tau.evalf().info(info_flags::numeric) && K.info(info_flags::posint) && C_norm.evalf().info(info_flags::numeric));
}

cln::cl_N Kronecker_dz_kernel::series_coeff_impl(int i) const
{
	numeric n_num = ex_to<numeric>(n);
	int n_int = n_num.to_int();

	ex w_j_inv = exp(ex_to<numeric>((-2*Pi*I*z_j).evalf()));
	cln::cl_N w_j_inv_cln = ex_to<numeric>(w_j_inv).to_cl_N();

	ex qbar = exp(ex_to<numeric>((2*Pi*I*K*tau).evalf()));

	// case n=0
	if ( n_num == 0 ) {
		return 0;
	}

	// case n=1
	if ( n_num == 1 ) {
		if ( i == 1 ) {
			return ex_to<numeric>((C_norm * 2*Pi*I).evalf()).to_cl_N();
		}

		return 0;
	}

	// case n=2
	if ( n_num == 2 ) {
		if ( ex_to<numeric>(z_j.evalf()).is_zero() ) {
			if ( i == 0 ) {
				return ex_to<numeric>((C_norm).evalf()).to_cl_N();
			}
			else if ( i == 1 ) {
				return 0;
			}
			else {
				ex res = -bernoulli(i)/numeric(i);
				if ( numeric(i).is_even() ) {
					Ebar_kernel Ebar = Ebar_kernel( 1-i, 0, numeric(1), numeric(1) );
					res += Ebar.get_numerical_value(qbar);
				}

				res *= -pow(2*Pi*I,i)*C_norm/factorial(i-1);

				return ex_to<numeric>(res.evalf()).to_cl_N();
			}
		}
		else {
			// z_j is not zero
			if ( i == 0 ) {
				return 0;
			}
			else {
				Li_negative my_Li_negative;

				ex res = 0;
				if ( i == 1 ) {
					res = numeric(1,2);
				}

				Ebar_kernel Ebar = Ebar_kernel( 1-i, 0, w_j_inv, numeric(1) );

				res += my_Li_negative.get_numerical_value(i-1,w_j_inv) + Ebar.get_numerical_value(qbar);

				res *= -pow(2*Pi*I,i)*C_norm/factorial(i-1);

				return ex_to<numeric>(res.evalf()).to_cl_N();
			}
		}
	}

	// case n>2
	ex res = 0;
	if ( i == 1 ) {
		res += - bernoulli(n_num-1)/(n_num-1);
	}
	if ( i > 0 ) {
		if ( ex_to<numeric>(z_j.evalf()).is_zero() ) {
			if ( (numeric(i)+n_num).is_even() ) {
				Ebar_kernel Ebar = Ebar_kernel( 1-i, 2-n_num, numeric(1), numeric(1) );

				res += pow(2*Pi*I,i-1)/factorial(i-1) * Ebar.get_numerical_value(qbar);
			}
		}
		else {
			// z_j is not zero
			Ebar_kernel Ebar = Ebar_kernel( 1-i, 2-n_num, w_j_inv, numeric(1) );

			res += pow(2*Pi*I,i-1)/factorial(i-1) * Ebar.get_numerical_value(qbar);
		}
	}

	res *= - C_norm * 2*Pi*I/factorial(n_num-2);

	return ex_to<numeric>(res.evalf()).to_cl_N();
}

/**
 *
 * Returns the value of the g^(n-1)(z-z_j,K*tau).
 *
 */
ex  Kronecker_dz_kernel::get_numerical_value(const ex & z, int N_trunc) const
{
	numeric n_num = ex_to<numeric>(n);

	if ( n_num == 1 ) {
		return 1;
	}

	ex pre = pow(2*Pi*I,n-2)/C_norm;

	return get_numerical_value_impl(z, pre, 0, N_trunc);
}

void Kronecker_dz_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "Kronecker_dz_kernel(";
	n.print(c);
	c.s << ",";
	z_j.print(c);
	c.s << ",";
	tau.print(c);
	c.s << ",";
	K.print(c);
	c.s << ",";
	C_norm.print(c);
	c.s << ")";
}

GINAC_BIND_UNARCHIVER(Kronecker_dz_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(Eisenstein_kernel, integration_kernel,
				     print_func<print_context>(&Eisenstein_kernel::do_print))

Eisenstein_kernel::Eisenstein_kernel() : inherited(), k(_ex0), N(_ex0), a(_ex0), b(_ex0), K(_ex0), C_norm(_ex1)
{ 
}

Eisenstein_kernel::Eisenstein_kernel(const ex & arg_k, const ex & arg_N, const ex & arg_a, const ex & arg_b, const ex & arg_K, const ex & arg_C_norm) : inherited(), k(arg_k), N(arg_N), a(arg_a), b(arg_b), K(arg_K), C_norm(arg_C_norm)
{ 
}

int Eisenstein_kernel::compare_same_type(const basic &other) const
{
	const Eisenstein_kernel &o = static_cast<const Eisenstein_kernel &>(other);
	int cmpval;

	cmpval = k.compare(o.k);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = N.compare(o.N);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = a.compare(o.a);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = b.compare(o.b);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = K.compare(o.K);
	if ( cmpval) {
		return cmpval;
	}

	return C_norm.compare(o.C_norm);
}

/**
 *
 * The series method for this class returns the qbar-expansion of the modular form, 
 * without an additional factor of C_norm/qbar.
 *
 * This allows for easy use in the class modular_form_kernel.
 *
 */
ex Eisenstein_kernel::series(const relational & r, int order, unsigned options) const
{
	if ( r.rhs() != 0 ) {
		throw (std::runtime_error("integration_kernel::series: non-zero expansion point not implemented"));
	}

	ex qbar = r.lhs();
	ex res = q_expansion_modular_form(qbar, order);
	res = res.series(qbar,order);

	return res;
}

size_t Eisenstein_kernel::nops() const
{
	return 6;
}

ex Eisenstein_kernel::op(size_t i) const
{
	switch (i) {
	case 0:
		return k;
	case 1:
		return N;
	case 2:
		return a;
	case 3:
		return b;
	case 4:
		return K;
	case 5:
		return C_norm;
	default:
		throw (std::out_of_range("Eisenstein_kernel::op() out of range"));
	}
}

ex & Eisenstein_kernel::let_op(size_t i)
{
	ensure_if_modifiable();

	switch (i) {
	case 0:
		return k;
	case 1:
		return N;
	case 2:
		return a;
	case 3:
		return b;
	case 4:
		return K;
	case 5:
		return C_norm;
	default:
		throw (std::out_of_range("Eisenstein_kernel::let_op() out of range"));
	}
}

bool Eisenstein_kernel::is_numeric(void) const
{
	return (k.info(info_flags::nonnegint) && N.info(info_flags::posint) && a.info(info_flags::integer) && b.info(info_flags::integer) && K.info(info_flags::posint) && C_norm.evalf().info(info_flags::numeric));
}

ex Eisenstein_kernel::Laurent_series(const ex & x, int order) const
{
	ex res = C_norm * q_expansion_modular_form(x, order+1)/x;
	res = res.series(x,order);

	return res;
}

/**
 *
 * Returns the value of the modular form.
 *
 */
ex  Eisenstein_kernel::get_numerical_value(const ex & qbar, int N_trunc) const
{
	ex pre = numeric(1)/C_norm;

	return get_numerical_value_impl(qbar, pre, 1, N_trunc);
}

bool Eisenstein_kernel::uses_Laurent_series() const
{
	return true;
}

ex Eisenstein_kernel::q_expansion_modular_form(const ex & q, int order) const
{
	numeric k_num = ex_to<numeric>(k);
	numeric N_num = ex_to<numeric>(N);
	numeric a_num = ex_to<numeric>(a);
	numeric b_num = ex_to<numeric>(b);
	numeric K_num = ex_to<numeric>(K);

	if ( (k==2) && (a==1) && (b==1) ) {
		return B_eisenstein_series(q, N_num, K_num, order);
	}

	return E_eisenstein_series(q, k_num, N_num, a_num, b_num, K_num, order);
}

void Eisenstein_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "Eisenstein_kernel(";
	k.print(c);
	c.s << ",";
	N.print(c);
	c.s << ",";
	a.print(c);
	c.s << ",";
	b.print(c);
	c.s << ",";
	K.print(c);
	c.s << ",";
	C_norm.print(c);
	c.s << ")";
}

GINAC_BIND_UNARCHIVER(Eisenstein_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(Eisenstein_h_kernel, integration_kernel,
				     print_func<print_context>(&Eisenstein_h_kernel::do_print))

Eisenstein_h_kernel::Eisenstein_h_kernel() : inherited(), k(_ex0), N(_ex0), r(_ex0), s(_ex0), C_norm(_ex1)
{ 
}

Eisenstein_h_kernel::Eisenstein_h_kernel(const ex & arg_k, const ex & arg_N, const ex & arg_r, const ex & arg_s, const ex & arg_C_norm) : inherited(), k(arg_k), N(arg_N), r(arg_r), s(arg_s), C_norm(arg_C_norm)
{ 
}

int Eisenstein_h_kernel::compare_same_type(const basic &other) const
{
	const Eisenstein_h_kernel &o = static_cast<const Eisenstein_h_kernel &>(other);
	int cmpval;

	cmpval = k.compare(o.k);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = N.compare(o.N);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = r.compare(o.r);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = s.compare(o.s);
	if ( cmpval) {
		return cmpval;
	}

	return C_norm.compare(o.C_norm);
}

/**
 *
 * The series method for this class returns the qbar-expansion of the modular form, 
 * without an additional factor of C_norm/qbar.
 *
 * This allows for easy use in the class modular_form_kernel.
 *
 */
ex Eisenstein_h_kernel::series(const relational & r, int order, unsigned options) const
{
	if ( r.rhs() != 0 ) {
		throw (std::runtime_error("integration_kernel::series: non-zero expansion point not implemented"));
	}

	ex qbar = r.lhs();
	ex res = q_expansion_modular_form(qbar, order);
	res = res.series(qbar,order);

	return res;
}

size_t Eisenstein_h_kernel::nops() const
{
	return 5;
}

ex Eisenstein_h_kernel::op(size_t i) const
{
	switch (i) {
	case 0:
		return k;
	case 1:
		return N;
	case 2:
		return r;
	case 3:
		return s;
	case 4:
		return C_norm;
	default:
		throw (std::out_of_range("Eisenstein_h_kernel::op() out of range"));
	}
}

ex & Eisenstein_h_kernel::let_op(size_t i)
{
	ensure_if_modifiable();

	switch (i) {
	case 0:
		return k;
	case 1:
		return N;
	case 2:
		return r;
	case 3:
		return s;
	case 4:
		return C_norm;
	default:
		throw (std::out_of_range("Eisenstein_h_kernel::let_op() out of range"));
	}
}

bool Eisenstein_h_kernel::is_numeric(void) const
{
	return (k.info(info_flags::nonnegint) && N.info(info_flags::posint) && r.info(info_flags::integer) && s.info(info_flags::integer) && C_norm.evalf().info(info_flags::numeric));
}

ex Eisenstein_h_kernel::Laurent_series(const ex & x, int order) const
{
	ex res = C_norm * q_expansion_modular_form(x, order+1)/x;
	res = res.series(x,order);

	return res;
}

/**
 *
 * Returns the value of the modular form.
 *
 */
ex  Eisenstein_h_kernel::get_numerical_value(const ex & qbar, int N_trunc) const
{
	ex pre = numeric(1)/C_norm;

	return get_numerical_value_impl(qbar, pre, 1, N_trunc);
}

bool Eisenstein_h_kernel::uses_Laurent_series() const
{
	return true;
}

/**
 *
 * The constant coefficient in the Fourier expansion.
 *
 */
ex Eisenstein_h_kernel::coefficient_a0(const numeric & k, const numeric & r, const numeric & s, const numeric & N) const
{
	if ( k == 1 ) {
		if ( irem(s,N) != 0 ) {
			return numeric(1,4) - mod(s,N)/numeric(2)/N;
		}
		else if ( (irem(r,N)==0) && (irem(s,N)==0) ) {
			return 0;
		}
		else {
			return I*numeric(1,4)*cos(Pi*mod(r,N)/N)/sin(Pi*mod(r,N)/N);
		}
	}

	// case k > 1
	return -Bernoulli_polynomial(k,mod(s,N)/N)/numeric(2)/k;
}

/**
 *
 * The higher coefficients in the Fourier expansion.
 *
 */
ex Eisenstein_h_kernel::coefficient_an(const numeric & n, const numeric & k, const numeric & r, const numeric & s, const numeric & N) const
{
	ex res = 0;

	for (numeric m=1; m<=n; m++) {
		if ( irem(n,m) == 0 ) {
			for (numeric c1=0; c1<N; c1++) {
				numeric c2 = n/m;
				res += pow(m,k-1)*exp(2*Pi*I/N*mod(r*c2-(s-m)*c1,N)) - pow(-m,k-1)*exp(2*Pi*I/N*mod(-r*c2+(s+m)*c1,N));
			}
		}
	}

	return res/numeric(2)/pow(N,k);
}

ex Eisenstein_h_kernel::q_expansion_modular_form(const ex & q, int N_order) const
{
	numeric N_order_num = numeric(N_order);

	numeric k_num = ex_to<numeric>(k);
	numeric r_num = ex_to<numeric>(r);
	numeric s_num = ex_to<numeric>(s);
	numeric N_num = ex_to<numeric>(N);

	ex res = coefficient_a0(k_num,r_num,s_num,N_num);

	for (numeric i1=1; i1<N_order_num; i1++) {
		res += coefficient_an(i1,k_num,r_num,s_num,N_num) * pow(q,i1);
	}

	res += Order(pow(q,N_order));
	res = res.series(q,N_order);

	return res;
}

void Eisenstein_h_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "Eisenstein_h_kernel(";
	k.print(c);
	c.s << ",";
	N.print(c);
	c.s << ",";
	r.print(c);
	c.s << ",";
	s.print(c);
	c.s << ",";
	C_norm.print(c);
	c.s << ")";
}

GINAC_BIND_UNARCHIVER(Eisenstein_h_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(modular_form_kernel, integration_kernel,
				     print_func<print_context>(&modular_form_kernel::do_print))

modular_form_kernel::modular_form_kernel() : inherited(), k(_ex0), P(_ex0), C_norm(_ex1)
{ 
}

modular_form_kernel::modular_form_kernel(const ex & arg_k, const ex & arg_P, const ex & arg_C_norm) : inherited(), k(arg_k), P(arg_P), C_norm(arg_C_norm)
{ 
}

int modular_form_kernel::compare_same_type(const basic &other) const
{
	const modular_form_kernel &o = static_cast<const modular_form_kernel &>(other);
	int cmpval;

	cmpval = k.compare(o.k);
	if ( cmpval) {
		return cmpval;
	}

	cmpval = P.compare(o.P);
	if ( cmpval) {
		return cmpval;
	}

	return C_norm.compare(o.C_norm);
}

/**
 *
 * The series method for this class returns the qbar-expansion of the modular form, 
 * without an additional factor of C_norm/qbar.
 *
 */
ex modular_form_kernel::series(const relational & r, int order, unsigned options) const
{
	if ( r.rhs() != 0 ) {
		throw (std::runtime_error("integration_kernel::series: non-zero expansion point not implemented"));
	}

	ex qbar = r.lhs();

	subs_q_expansion do_subs_q_expansion(qbar, order);

	ex res = do_subs_q_expansion(P).series(qbar,order);
	res += Order(pow(qbar,order));
	res = res.series(qbar,order);

	return res;
}

size_t modular_form_kernel::nops() const
{
	return 3;
}

ex modular_form_kernel::op(size_t i) const
{
	switch (i) {
	case 0:
		return k;
	case 1:
		return P;
	case 2:
		return C_norm;
	default:
		throw (std::out_of_range("modular_form_kernel::op() out of range"));
	}
}

ex & modular_form_kernel::let_op(size_t i)
{
	ensure_if_modifiable();

	switch (i) {
	case 0:
		return k;
	case 1:
		return P;
	case 2:
		return C_norm;
	default:
		throw (std::out_of_range("modular_form_kernel::let_op() out of range"));
	}
}

bool modular_form_kernel::is_numeric(void) const
{
	bool flag = (k.info(info_flags::nonnegint) && C_norm.evalf().info(info_flags::numeric));
	if ( !flag ) {
		return false;
	}

	symbol qbar("qbar");

	// test with a random number and random expansion
	return series_to_poly(q_expansion_modular_form(qbar,18)).subs(qbar==numeric(1,937)).evalf().info(info_flags::numeric);
}

ex modular_form_kernel::Laurent_series(const ex & qbar, int order) const
{
	ex res = series_to_poly(q_expansion_modular_form(qbar,order+1));
	res = C_norm * res/qbar;
	res = res.series(qbar,order);
	return res;
}

/**
 *
 * Returns the value of the modular form.
 *
 */
ex  modular_form_kernel::get_numerical_value(const ex & qbar, int N_trunc) const
{
	ex pre = numeric(1)/C_norm;

	return get_numerical_value_impl(qbar, pre, 1, N_trunc);
}

bool modular_form_kernel::uses_Laurent_series() const
{
	return true;
}

ex modular_form_kernel::q_expansion_modular_form(const ex & q, int N_order) const
{
	return this->series(q==0,N_order);
}

void modular_form_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "modular_form_kernel(";
	k.print(c);
	c.s << ",";
	P.print(c);
	c.s << ",";
	C_norm.print(c);
	c.s << ")";
}

GINAC_BIND_UNARCHIVER(modular_form_kernel);


GINAC_IMPLEMENT_REGISTERED_CLASS_OPT(user_defined_kernel, integration_kernel,
				     print_func<print_context>(&user_defined_kernel::do_print))

user_defined_kernel::user_defined_kernel() : inherited(), f(_ex0), x(_ex0)
{ 
}

user_defined_kernel::user_defined_kernel(const ex & arg_f, const ex & arg_x) : inherited(), f(arg_f), x(arg_x)
{ 
}

int user_defined_kernel::compare_same_type(const basic &other) const
{
	const user_defined_kernel &o = static_cast<const user_defined_kernel &>(other);
	int cmpval;

	cmpval = f.compare(o.f);
	if ( cmpval) {
		return cmpval;
	}

	return x.compare(o.x);
}

size_t user_defined_kernel::nops() const
{
	return 2;
}

ex user_defined_kernel::op(size_t i) const
{
	switch (i) {
	case 0:
		return f;
	case 1:
		return x;
	default:
		throw (std::out_of_range("user_defined_kernel::op() out of range"));
	}
}

ex & user_defined_kernel::let_op(size_t i)
{
	ensure_if_modifiable();

	switch (i) {
	case 0:
		return f;
	case 1:
		return x;
	default:
		throw (std::out_of_range("user_defined_kernel::let_op() out of range"));
	}
}

bool user_defined_kernel::is_numeric(void) const
{
	// test with a random number
	return f.subs(x==numeric(1,937)).evalf().info(info_flags::numeric);
}

ex user_defined_kernel::Laurent_series(const ex & x_up, int order) const
{
	ex res = f.series(x,order).subs(x==x_up);

	return res;
}

bool user_defined_kernel::uses_Laurent_series() const
{
	return true;
}

void user_defined_kernel::do_print(const print_context & c, unsigned level) const
{
	c.s << "user_defined_kernel(";
	f.print(c);
	c.s << ",";
	x.print(c);
	c.s << ")";
}

GINAC_BIND_UNARCHIVER(user_defined_kernel);

} // namespace GiNaC
