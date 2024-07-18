/** @file exam_factor.cpp
 *
 *  Factorization test suite. */

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
using namespace std;

static unsigned check_factor(const ex& e)
{
	ex ee = e.expand();
	ex answer = factor(ee);
	if ( answer.expand() != ee || answer != e ) {
		clog << "factorization of " << e << " == " << ee << " gave wrong result: " << answer << endl;
		return 1;
	}
	return 0;
}

static unsigned exam_factor1()
{
	unsigned result = 0;
	ex e;
	symbol x("x");
	lst syms = {x};

	e = 1;
	result += check_factor(e);

	e = ex("1+x-x^3", syms);
	result += check_factor(e);

	e = ex("1+x^6+x", syms);
	result += check_factor(e);

	e = ex("1-x^6+x", syms);
	result += check_factor(e);

	e = ex("(1+x)^3", syms);
	result += check_factor(e);

	e = ex("(x+1)*(x+4)", syms);
	result += check_factor(e);

	e = ex("x^6-3*x^5+x^4-3*x^3-x^2-3*x+1", syms);
	result += check_factor(e);

	e = ex("(-1+x)^3*(1+x)^3*(1+x^2)", syms);
	result += check_factor(e);

	e = ex("-(-168+20*x-x^2)*(30+x)", syms);
	result += check_factor(e);

	e = ex("x^2*(x-3)^2*(x^3-5*x+7)", syms);
	result += check_factor(e);

	e = ex("-6*x^2*(x-3)", syms);
	result += check_factor(e);

	e = ex("x^16+11*x^4+121", syms);
	result += check_factor(e);

	e = ex("x^8-40*x^6+352*x^4-960*x^2+576", syms);
	result += check_factor(e);

	e = ex("x*(2+x^2)*(1+x+x^3+x^2+x^6+x^5+x^4)*(1+x)^2*(1-x+x^2)^2*(-1+x)", syms);
	result += check_factor(e);

	e = ex("(x+4+x^2-x^3+43*x^4)*(x+1-x^2-3*x^3+4*x^4)", syms);
	result += check_factor(e);

	e = ex("-x^2*(x-1)*(1+x^2)", syms);
	result += check_factor(e);

	e = x;
	result += check_factor(e);

	// x^37 + 1
	e = ex("(1+x)*(1+x^2-x^29-x^11-x^25-x^9-x^35+x^20-x^3+x^16-x^15-x-x^13+x^28+x^24-x^33+x^8-x^19+x^36+x^12-x^27+x^10-x^23+x^18+x^14+x^34-x^31+x^32+x^30-x^5+x^26+x^4+x^22-x^21-x^7-x^17+x^6)", syms);
	result += check_factor(e);

	e = ex("(1+4*x)*x^2*(1-4*x+16*x^2)*(3+5*x+92*x^3)", syms);
	result += check_factor(e);

	e = ex("(77+11*x^3+25*x^2+27*x+102*x^4)*(85+57*x^3+92*x^2+29*x+66*x^4)", syms);
	result += check_factor(e);

	return result;
}

static unsigned exam_factor2()
{
	unsigned result = 0;
	ex e;
	symbol x("x"), y("y"), z("z");
	lst syms = {x, y, z};
	
	e = ex("x+y", syms);
	result += check_factor(e);

	e = ex("(x^2-y+1)*(x+y)", syms);
	result += check_factor(e);

	e = ex("-2*(x+y)*(x-y)", syms);
	result += check_factor(e);

	e = ex("(16+x^2*z^3)*(-17+3*x-5*z)*(2*x+3*z)*(x-y^2-z^3)", syms);
	result += check_factor(e);

	e = ex("(x-y*z)*(x-y^2-z^3)*(x+y+z)", syms);
	result += check_factor(e);
	
	e = ex("-(y^2-x+z^3)*x*(x+y+z)", syms);
	result += check_factor(e);
	
	e = ex("-316*(3*x-4*z)*(2*x+3*z)*(x+y)*(-1+x)", syms);
	result += check_factor(e);
	
	e = ex("(x+x^3+z^2)*(3*x-4*z)", syms);
	result += check_factor(e);
	
	e = ex("250*(-3+x)*(4*z-3*x)*(x^3+z^2+x)*x", syms);
	result += check_factor(e);
	
	e = ex("327*(x+z^2+x^3)*(3*x-4*z)*(-7+5*x-x^3)*(1+x+x^2)", syms);
	result += check_factor(e);
	
	e = ex("x-y^2-z^3", syms);
	result += check_factor(e);
	
	e = ex("-390*(7+3*x^4)*(2+x^2)*(x-z^3-y^2)", syms);
	result += check_factor(e);
	
	e = ex("55*(1+x)^2*(3*x-4*z)*(1+x+x^2)*(x+x^3+z^2)", syms);
	result += check_factor(e);
	
	e = ex("x+y*x-1", syms);
	result += check_factor(e);
	
	e = ex("390*(-1+x^6-x)*(7+3*x^4)*(2+x^2)*(y+x)*(-1+y-x^2)*(1+x^2+x)^2", syms);
	result += check_factor(e);
	
	e = ex("310*(y+x)*(-1+y-x^2)", syms);
	result += check_factor(e);

	return result;
}

static unsigned exam_factor3()
{
	unsigned result = 0;
	ex e;
	symbol k("k"), n("n");
	lst syms = {k, n};
	
	e = ex("1/2*(-3+3*k-n)*(-2+3*k-n)*(-1+3*k-n)", syms);
	result += check_factor(e);

	e = ex("1/4*(2*k-n)*(-1+2*k-n)", syms);
	result += check_factor(e);

	return result;
}

static unsigned check_factor_expanded(const ex& e)
{
	ex ee = e.expand();
	ex answer = factor(ee);
	if ( answer.expand() != ee || (!is_a<mul>(answer) && !is_a<power>(answer)) ) {
		clog << "factorization of " << e << " == " << ee << " gave wrong result: " << answer << endl;
		return 1;
	}
	return 0;
}

static unsigned exam_factor_content()
{
	unsigned result = 0;
	ex e;
	symbol x("x"), y("y");

	// Fixed 2013-07-28 by Alexei Sheplyakov in factor_univariate().
	e = ex("174247781*x^2-1989199947807987/200000000000000", lst{x});
	result += check_factor(e);

	// Fixed 2014-05-18 by Alexei Sheplyakov in factor_multivariate().
	e = ex("(x+y+x*y)*(3*x+2*y)", lst{x, y});
	result += check_factor(e);

	return result;
}

static unsigned exam_factor_wang()
{
	// these 15 polynomials are from the appendix of P.S.Wang,
	// "An Improved Multivariate Polynomial Factoring Algorithm"
	unsigned result = 0;
	ex e;
	symbol u("u"), w("w"), x("x"), y("y"), z("z");

	e = ex("(z+x*y+10)*(x*z+y+30)*(y*z+x+20)", lst{x, y, z});
	result += check_factor_expanded(e);

	e = ex("(x^3*(z+y)+y-11)*(x^2*(z^2+y^2)+y+90)", lst{x, y, z});
	result += check_factor_expanded(e);

	e = ex("(y*z^3+x*y*z+y^2+x^3)*(x*(z^4+1)+z+x^3*y^2)", lst{x, y, z});
	result += check_factor_expanded(e);

	e = ex("(z^2-x^3*y+3)*(z^2+x*y^3)*(z^2+x^3*y^4)*(y^4*z^2+x^2*z+5)", lst{x, y, z});
	result += check_factor_expanded(e);

	e = ex("(z^2+x^3*y^4+u^2)*((y^2+x)*z^2+3*u^2*x^3*y^4*z+19*y^2)*(u^2*y^4*z^2+x^2*z+5)", lst{u, x, y, z});
	result += check_factor_expanded(e);

	e = ex("(w^4*z^3-x*y^2*z^2-w^4*x^5*y^6-w^2*x^3*y)*(-x^5*z^3+y*z+x^2*y^3)"
	       "*(w^4*z^6+y^2*z^3-w^2*x^2*y^2*z^2+x^5*z-x^4*y^2-w^3*x^3*y)", lst{w, x, y, z});
	result += check_factor_expanded(e);

	e = ex("(z+y+x-3)^3*(z+y+x-2)^2", lst{x, y, z});
	result += check_factor_expanded(e);

	e = ex("(-15*y^2*z^16+29*w^4*x^12*y^12*z^3+21*x^3*z^2+3*w^15*y^20)"
	       "*(-z^31-w^12*z^20+y^18-y^14+x^2*y^2+x^21+w^2)", lst{w, x, y, z});
	result += check_factor_expanded(e);

	e = ex("u^4*x*z^2*(6*w^2*y^3*z^2+18*u^2*w^3*x*z^2+15*u*z^2+10*u^2*w*x*y^3)"
	       "*(-44*u*w*x*y^4*z^4-25*u^2*w^3*y*z^4+8*u*w*x^3*z^4-32*u^2*w^4*y^4*z^3"
	       "+48*u^2*x^2*y^3*z^3-12*y^3*z^2+2*u^2*w*x^2*y^2-11*u*w^2*x^3*y-4*w^2*x)", lst{u, w, x, y, z});
	result += check_factor_expanded(e);

	e = ex("(31*u^2*x*z+35*w^2*y^2+6*x*y+40*w*x^2)*(u^2*w^2*x*y^2*z^2+24*u^2*w*x*y^2*z^2"
	       "+12*u^2*x*y^2*z^2+24*u^2*x^2*y*z^2+43*w*x*y*z^2+31*w^2*y*z^2+8*u^2*w^2*z^2"
	       "+44*u*w^2*z^2+37*u^2*y^2*z+41*y^2*z+12*w*x^2*y*z+21*u^2*w*x*y*z+23*x*y*z"
	       "+47*u^2*w^2*z+13*u*w^2*x^2*y^2+22*x*y^2+42*u^2*w^2*y^2+29*w^2*y^2+27*u*w^2*x^2*y"
	       "+37*w^2*x*z+39*u*w*x*z+43*u*x^2*y+24*x*y+9*u^2*w*x^2+22*u^2*w^2)", lst{u, w, x, y, z});
	result += check_factor_expanded(e);

	e = ex("x*y*(-13*u^3*w^2*x*y*z^3+w^3*z^3+4*u*x*y^2+47*x*y)"
	       "*(43*u*x^3*y^3*z^3+36*u^2*w^3*x*y*z^3+14*w^3*x^3*y^3*z^2-29*w^3*x*y^3*z^2"
	       "-20*u^2*w^2*x^2*y^2*z^2+36*u^2*w*x*y^3*z-48*u*x^3*y^2*z+5*u*w*x^2*y^3"
	       "+36*u*w^2*y^3-9*u*w*y^3-23*u*w*x^3*y^2+46*u*x^3*y^2+8*x*y^2+31*u^2*w^3*y^2"
	       "-9*u^2*y^2+45*x^3-46*u^2*w*x)", lst{u, w, x, y, z});
	result += check_factor_expanded(e);

	e = ex("(z+y+x-3)^3", lst{x, y, z});
	result += check_factor_expanded(e);

	e = ex("(3*z^3+2*w*z-9*y^3-y^2+45*x^3)*(w^2*z^3+47*x*y-w^2)", lst{w, x, y, z});
	result += check_factor_expanded(e);

	e = ex("(-18*x^4*y^5+22*y^5-26*x^3*y^4-38*x^2*y^4+29*x^2*y^3-41*x^4*y^2+37*x^4)"
	       "*(33*x^5*y^6+11*y^2+35*x^3*y-22*x^4)", lst{x, y, z});
	result += check_factor_expanded(e);

	e = ex("x^6*y^3*z^2*(3*z^3+2*w*z-8*x*y^2+14*w^2*y^2-y^2+18*x^3*y)"
	       "*(-12*w^2*x*y*z^3+w^2*z^3+3*x*y^2+29*x-w^2)", lst{w, x, y, z});
	result += check_factor_expanded(e);

	return result;
}

static unsigned exam_factor_magerya()
{
	// In 2017, Vitaly Magerya reported a class of biviariate polynomials
	// where Hensel lifting sometimes failed to terminate.
	// https://www.ginac.de/pipermail/ginac-list/2017-December/002162.html
	unsigned result = 0;
	ex e;
	symbol x("x"), y("y");

	e = (1+2*x+y)*(1+2*x-y)*(2*x-y)*(2*x+y);
	result += check_factor_expanded(e);

	e = (7+4*x+y)*(-5+2*x-y)*(-6+6*x+y)*y*(10+2*x+y);
	result += check_factor_expanded(e);

	e = (8+6*x-y)*(-5+4*x-y)*(-5+6*x+y)*(-2+2*x-y)*(2+4*x+y);
	result += check_factor_expanded(e);

	e = -(-4+4*x+5*y)*(1+4*x+5*y)*(2+3*y)*(1+2*x-y)*(4+2*x+y);
	result += check_factor_expanded(e);

	e = (-3+y-2*x)*(4+3*y-4*x)*(3+3*y+2*x)*(-2+3*y+2*x)*(-1+4*y+3*x);
	result += check_factor_expanded(e);

	e = (-9+7*x+y)*(-5+6*x+y)*(4+2*x+y)*(5+2*x-y)*(7+9*x-y)*(8+6*x-y);
	result += check_factor_expanded(e);

	e = pow(2*x-y,2)*(-1+6*x-y)*(-1+3*x-y)*(-2+4*x-y)*(1+4*x-y)*(4*x-y)*(2+4*x-y);
	result += check_factor_expanded(e);

	e = (5+2*y-3*x)*(-4+4*y+3*x)*(-3+4*y-2*x)*(4+5*y-x)*(3*y+2*x)*(-1+3*y+5*x)*(5+3*y+4*x);
	result += check_factor_expanded(e);

	return result;
}

unsigned exam_factor()
{
	unsigned result = 0;

	cout << "examining polynomial factorization" << flush;

	result += exam_factor1(); cout << '.' << flush;
	result += exam_factor2(); cout << '.' << flush;
	result += exam_factor3(); cout << '.' << flush;
	result += exam_factor_content(); cout << '.' << flush;
	result += exam_factor_wang(); cout << '.' << flush;
	result += exam_factor_magerya(); cout << '.' << flush;

	return result;
}

int main(int argc, char** argv)
{
	return exam_factor();
}
