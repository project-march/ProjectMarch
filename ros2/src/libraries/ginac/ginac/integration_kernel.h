/** @file integration_kernel.h
 *
 *  Interface to GiNaC's integration kernels for iterated integrals. */

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

#ifndef GINAC_INTEGRATION_KERNEL_H
#define GINAC_INTEGRATION_KERNEL_H

#include "basic.h"
#include "archive.h"
#include "numeric.h"
#include "lst.h"

#include <cln/complex.h>
#include <vector>

namespace GiNaC {

ex ifactor(const numeric & n);
bool is_discriminant_of_quadratic_number_field(const numeric & n);
numeric kronecker_symbol(const numeric & a, const numeric & n);
numeric primitive_dirichlet_character(const numeric & n, const numeric & a);
numeric dirichlet_character(const numeric & n, const numeric & a, const numeric & N);
numeric generalised_Bernoulli_number(const numeric & k, const numeric & b);
ex Bernoulli_polynomial(const numeric & k, const ex & x);

/**
 *
 * The base class for integration kernels for iterated integrals.
 *
 * This class represents the differential one-form
 * \f[
 *    \omega = d\lambda
 * \f]
 * The integration variable is a dummy variable and does not need to be specified.
 *
 */
class integration_kernel : public basic
{
	GINAC_DECLARE_REGISTERED_CLASS(integration_kernel, basic)

	// ctors
public:

	// functions overriding virtual functions from base classes
public:
	ex series(const relational & r, int order, unsigned options = 0) const override;

protected:

	// new virtual functions which can be overridden by derived classes
public:
	virtual bool has_trailing_zero(void) const;
	virtual bool is_numeric(void) const;
	virtual ex Laurent_series(const ex & x, int order) const;
	virtual ex  get_numerical_value(const ex & lambda, int N_trunc = 0) const;

protected:
	virtual bool uses_Laurent_series() const;
	virtual cln::cl_N series_coeff_impl(int i) const;

	// non-virtual functions 
public:
	size_t get_cache_size(void) const;
	void set_cache_step(int cache_steps) const;
	ex get_series_coeff(int i) const;
	cln::cl_N series_coeff(int i) const;

protected:
	ex  get_numerical_value_impl(const ex & lambda, const ex & pre, int shift, int N_trunc) const;
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	// cache is increased by steps of cache_step_size
	mutable int cache_step_size;
	// cache already computed series coefficients
	mutable std::vector<cln::cl_N> series_vec;

};

GINAC_DECLARE_UNARCHIVER(integration_kernel);

/**
 *
 * The basic integration kernel with a logarithmic singularity at the origin.
 *
 * This class represents the differential one-form
 * \f[
 *    L_0 = \frac{d\lambda}{\lambda}
 * \f]
 *
 */
class basic_log_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(basic_log_kernel, integration_kernel)

	// ctors
public:

	// functions overriding virtual functions from base classes
public:

protected:
	cln::cl_N series_coeff_impl(int i) const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:

protected:
	void do_print(const print_context & c, unsigned level) const;

	// friends :

	// member variables :

protected:

};

GINAC_DECLARE_UNARCHIVER(basic_log_kernel);

/**
 *
 * The integration kernel for multiple polylogarithms.
 *
 * This class represents the differential one-form
 * \f[
 *    \omega^{\mathrm{mpl}}(z) = \frac{d\lambda}{\lambda-z}
 * \f]
 *
 * For the case \f$ z=0 \f$ the class basic_log_kernel should be used.
 *
 */
class multiple_polylog_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(multiple_polylog_kernel, integration_kernel)

	// ctors
public:
	multiple_polylog_kernel(const ex & z);

	// functions overriding virtual functions from base classes
public:
	size_t nops() const override;
	ex op(size_t i) const override;
	ex & let_op(size_t i) override;

	bool is_numeric(void) const override;

protected:
	cln::cl_N series_coeff_impl(int i) const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:

protected:
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	ex z;

};

GINAC_DECLARE_UNARCHIVER(multiple_polylog_kernel);

/**
 *
 * The ELi-kernel.
 *
 * This class represents the differential one-form
 * \f[
 *    \omega^{\mathrm{ELi}}_{n;m}(x;y) = \mathrm{ELi}_{n;m}(x;y;\bar{q}) \frac{d\bar{q}}{\bar{q}}
 * \f]
 *
 */
class ELi_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(ELi_kernel, integration_kernel)

	// ctors
public:
	ELi_kernel(const ex & n, const ex & m, const ex & x, const ex & y);

	// functions overriding virtual functions from base classes
public:
	size_t nops() const override;
	ex op(size_t i) const override;
	ex & let_op(size_t i) override;

	bool is_numeric(void) const override;
	ex get_numerical_value(const ex & qbar, int N_trunc = 0) const override;

protected:
	cln::cl_N series_coeff_impl(int i) const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:

protected:
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	ex n;
	ex m;
	ex x;
	ex y;

};

GINAC_DECLARE_UNARCHIVER(ELi_kernel);

/**
 *
 * The Ebar-kernel
 *
 * This class represents the differential one-form
 * \f[
 *  \omega^{\overline{\mathrm{E}}}_{n;m}(x;y) = \overline{\mathrm{E}}_{n;m}(x;y;\bar{q}) \frac{d\bar{q}}{\bar{q}}
 * \f]
 *
 */
class Ebar_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(Ebar_kernel, integration_kernel)

	// ctors
public:
	Ebar_kernel(const ex & n, const ex & m, const ex & x, const ex & y);

	// functions overriding virtual functions from base classes
public:
	size_t nops() const override;
	ex op(size_t i) const override;
	ex & let_op(size_t i) override;

	bool is_numeric(void) const override;
	ex get_numerical_value(const ex & qbar, int N_trunc = 0) const override;

protected:
	cln::cl_N series_coeff_impl(int i) const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:

protected:
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	ex n;
	ex m;
	ex x;
	ex y;

};

GINAC_DECLARE_UNARCHIVER(Ebar_kernel);

/**
 *
 * The kernel corresponding to integrating the Kronecker coefficient function \f$ g^{(n)}(z_j,K \tau) \f$ 
 * in \f$ \tau \f$ (or equivalently in \f$ \bar{q} \f$).
 *
 * This class represents the differential one-form
 * \f[
 *  \omega^{\mathrm{Kronecker},\tau}_{n,K}(z_j) = \frac{C_n K (n-1)}{(2\pi i)^n} g^{(n)}(z_j,K \tau) \frac{d\bar{q}}{\bar{q}}
 * \f]
 *
 */
class Kronecker_dtau_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(Kronecker_dtau_kernel, integration_kernel)

	// ctors
public:
	Kronecker_dtau_kernel(const ex & n, const ex & z, const ex & K = numeric(1), const ex & C_norm = numeric(1));

	// functions overriding virtual functions from base classes
public:
	size_t nops() const override;
	ex op(size_t i) const override;
	ex & let_op(size_t i) override;

	bool is_numeric(void) const override;
	ex get_numerical_value(const ex & qbar, int N_trunc = 0) const override;

protected:
	cln::cl_N series_coeff_impl(int i) const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:

protected:
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	ex n;
	ex z;
	ex K;
	ex C_norm;
};

GINAC_DECLARE_UNARCHIVER(Kronecker_dtau_kernel);


/**
 *
 * The kernel corresponding to integrating the Kronecker coefficient function \f$ g^{(n-1)}(z-z_j, K \tau) \f$ 
 * in \f$ z \f$.
 *
 * This class represents the differential one-form
 * \f[
 *   \omega^{\mathrm{Kronecker},z}_{n,K}(z_j,\tau) = C_n (2\pi i)^{2-n} g^{(n-1)}(z-z_j, K \tau) dz
 * \f]
 *
 */
class Kronecker_dz_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(Kronecker_dz_kernel, integration_kernel)

	// ctors
public:
	Kronecker_dz_kernel(const ex & n, const ex & z_j, const ex & tau, const ex & K = numeric(1), const ex & C_norm = numeric(1));

	// functions overriding virtual functions from base classes
public:
	size_t nops() const override;
	ex op(size_t i) const override;
	ex & let_op(size_t i) override;

	bool is_numeric(void) const override;
	ex get_numerical_value(const ex & z, int N_trunc = 0) const override;

protected:
	cln::cl_N series_coeff_impl(int i) const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:

protected:
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	ex n;
	ex z_j;
	ex tau;
	ex K;
	ex C_norm;

};

GINAC_DECLARE_UNARCHIVER(Kronecker_dz_kernel);


/**
 *
 * The kernel corresponding to the Eisenstein series \f$ E_{k,N,a,b,K}(\tau) \f$.
 *
 * This class represents the differential one-form
 * \f[
 *   \omega^{\mathrm{Eisenstein}}_{k,N,a,b,K} = C_k E_{k,N,a,b,K}(\tau) \frac{d\bar{q}_N}{\bar{q}_N}
 * \f]
 *
 * The integers a and b are either one or the discriminant of a quadratic number field.
 * This class represents Eisenstein series, which can be defined by primitive Dirichlet characters from the Kronecker symbol.
 * This implies that the characters take the values -1,0,1, i.e. no higher roots of unity occur.
 * The \f[ \bar{q} \f]-expansion has then rational coefficients.
 *
 * Ref.: W. Stein, Modular Forms: A Computational Approach, Chapter 5
 *
 */
class Eisenstein_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(Eisenstein_kernel, integration_kernel)

	// ctors
public:
	Eisenstein_kernel(const ex & k, const ex & N, const ex & a, const ex & b, const ex & K, const ex & C_norm = numeric(1));

	// functions overriding virtual functions from base classes
public:
	ex series(const relational & r, int order, unsigned options = 0) const override;

	size_t nops() const override;
	ex op(size_t i) const override;
	ex & let_op(size_t i) override;

	bool is_numeric(void) const override;
	ex Laurent_series(const ex & x, int order) const override;
	ex get_numerical_value(const ex & qbar, int N_trunc = 0) const override;

protected:
	bool uses_Laurent_series() const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:
	ex q_expansion_modular_form(const ex & q, int order) const;

protected:
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	ex k;
	ex N;
	ex a;
	ex b;
	ex K;
	ex C_norm;

};

GINAC_DECLARE_UNARCHIVER(Eisenstein_kernel);


/**
 *
 * The kernel corresponding to the Eisenstein series \f$ h_{k,N,r,s}(\tau) \f$.
 *
 * This class represents the differential one-form
 * \f[
 *   \omega^{\mathrm{Eisenstein,h}}_{k,N,r,s} = C_k h_{k,N,r,s}(\tau) \frac{d\bar{q}_N}{\bar{q}_N}
 * \f]
 *
 */
class Eisenstein_h_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(Eisenstein_h_kernel, integration_kernel)

	// ctors
public:
	Eisenstein_h_kernel(const ex & k, const ex & N, const ex & r, const ex & s, const ex & C_norm = numeric(1));

	// functions overriding virtual functions from base classes
public:
	ex series(const relational & r, int order, unsigned options = 0) const override;

	size_t nops() const override;
	ex op(size_t i) const override;
	ex & let_op(size_t i) override;

	bool is_numeric(void) const override;
	ex Laurent_series(const ex & x, int order) const override;
	ex get_numerical_value(const ex & qbar, int N_trunc = 0) const override;

protected:
	bool uses_Laurent_series() const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:
	ex coefficient_a0(const numeric & k, const numeric & r, const numeric & s, const numeric & N) const;
	ex coefficient_an(const numeric & n, const numeric & k, const numeric & r, const numeric & s, const numeric & N) const;
	ex q_expansion_modular_form(const ex & q, int order) const;

protected:
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	ex k;
	ex N;
	ex r;
	ex s;
	ex C_norm;

};

GINAC_DECLARE_UNARCHIVER(Eisenstein_h_kernel);


/**
 *
 * A kernel corresponding to a polynomial in Eisenstein series.
 *
 * This class represents the differential one-form
 * \f[
 *   \omega^{\mathrm{modular}}(P_k(\eta^{(1)}_{k_1}, \dots, \eta^{(r)}_{k_r})) = C_k P_k(\eta^{(1)}_{k_1}, \dots, \eta^{(r)}_{k_r}) \frac{d\bar{q}_N}{\bar{q}_N}.
 * \f]
 *
 */
class modular_form_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(modular_form_kernel, integration_kernel)

	// ctors
public:
	modular_form_kernel(const ex & k, const ex & P, const ex & C_norm = numeric(1));

	// functions overriding virtual functions from base classes
public:
	ex series(const relational & r, int order, unsigned options = 0) const override;

	size_t nops() const override;
	ex op(size_t i) const override;
	ex & let_op(size_t i) override;

	bool is_numeric(void) const override;
	ex Laurent_series(const ex & qbar, int order) const override;
	ex get_numerical_value(const ex & qbar, int N_trunc = 0) const override;

protected:
	bool uses_Laurent_series() const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:
	ex q_expansion_modular_form(const ex & q, int order) const;

protected:
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	ex k;
	ex P;
	ex C_norm;

};

GINAC_DECLARE_UNARCHIVER(modular_form_kernel);


/**
 *
 * A user-defined integration kernel.
 * The input is an expression \f$ f \f$, depending on a variable \f$ x \f$.
 * It is assumed that \f$ f \f$ has a Laurent expansion around \f$ x=0 \f$ and 
 * maximally a simple pole at \f$ x=0 \f$.
 *
 */
class user_defined_kernel : public integration_kernel
{
	GINAC_DECLARE_REGISTERED_CLASS(user_defined_kernel, integration_kernel)

	// ctors
public:
	user_defined_kernel(const ex & f, const ex & x);

	// functions overriding virtual functions from base classes
public:
	size_t nops() const override;
	ex op(size_t i) const override;
	ex & let_op(size_t i) override;

	bool is_numeric(void) const override;
	ex Laurent_series(const ex & x, int order) const override;

protected:
	bool uses_Laurent_series() const override;

	// new virtual functions which can be overridden by derived classes
public:

protected:

	// non-virtual functions 
public:

protected:
	void do_print(const print_context & c, unsigned level) const;

        // friends :

	// member variables :

protected:
	ex f;
	ex x;

};

GINAC_DECLARE_UNARCHIVER(user_defined_kernel);

} // namespace GiNaC

#endif // ndef GINAC_INTEGRATION_KERNEL_H
