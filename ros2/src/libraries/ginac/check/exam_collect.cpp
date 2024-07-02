/** @file exam_collect.cpp
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

#include "ginac.h"
using namespace GiNaC;

#include <iostream>
using namespace std;

// Correctness of .collect(Z[x], x).
static unsigned exam_collect_1()
{
	unsigned result = 0;
	symbol x("x"), y("y");
	ex a = (pow(x, 3) - 2*pow(x, 2) + 4*x) * pow(y, 2)
	       + (pow(x, 2) - x - 1) * y
	       + (x + 1);
	ex b = pow(y, 2) * pow(x, 3)
	       + (y - 2*pow(y, 2)) * pow(x, 2)
	       + (4*pow(y, 2) - y + 1) * x
	       + (1 - y);

	ex a_x = collect(a, x);
	if (a_x != b) {
		clog << "collect(" << a << ", " << x << ") erroneously returned "
		     << a_x << " instead of " << b << endl;
		++result;
	}

	ex b_y = collect(b, y);
	if (b_y != a) {
		clog << "collect(" << b << ", " << y << ") erroneously returned "
		     << b_y << " instead of " << a << endl;
		++result;
	}

	ex amb_x = collect(a - b, x);
	if (amb_x != 0) {
		clog << "collect(" << a - b << ", " << x << ") erroneously returned "
		     << amb_x << " instead of 0" << endl;
		++result;
	}

	ex amb_y = collect(a - b, y);
	if (amb_y != 0) {
		clog << "collect(" << a - b << ", " << y << ") erroneously returned "
		     << amb_y << " instead of 0" << endl;
		++result;
	}

	return result;
}

// Consistency of .collect(Z[x,y], {x,y}) with .coeff(x).
static unsigned exam_collect_2()
{
	unsigned result = 0;
	symbol x("x"), y("y"), p("p"), q("q");

	ex e1 = x + y;
	ex e2 = 1 + p + q;
	ex a = expand(e1 * e2);

	ex a_x = a.collect(x).coeff(x, 1);
	if (a_x != e2) {
		clog << "collect(" << a << ", " << x << ") erroneously returned "
		     << a_x << " as coefficient of " << x << endl;
		++result;
	}

	ex a_p = a.collect(p).coeff(p, 1);
	if (a_p != e1) {
		clog << "collect(" << a << ", " << p << ") erroneously returned "
		     << a_p << " as coefficient of " << p << endl;
		++result;
	}

	ex a_xy = a.collect(lst{x,y});
	ex ref = e2*x + e2*y;
	if (a_xy != ref) {
		clog << "collect(" << a << ", {" << x << ", " << y << "}) erroneously returned "
		     << a_xy << " instead of " << ref << endl;
		++result;
	}

	return result;
}

// Consistency of .collect(Z[f(x)], f(x)) with .coeff(f(x)).
static unsigned exam_collect_3()
{
	unsigned result = 0;
	symbol x("x"), p("p"), q("q");

	for (unsigned deg = 2; deg < 7; ++deg) {

		ex a1 = expand(pow(p + q + x, deg));
		a1 = a1.collect(x);

		ex a2 = expand(pow(p + q + sin(x), deg));
		a2 = a2.collect(sin(x));

		for (unsigned i = 0; i < deg; ++i) {
			ex a1_i = a1.coeff(x, i);
			ex a2_i = a2.coeff(sin(x), i);
			if (!expand(a1_i - a2_i).is_zero()) {
				clog << "collect(" << a1 << ",sin(x)) inconsistent with "
					"collect(" << a2 << ",x)" << endl;
				++result;
			}
		}
	}

	return result;
}

unsigned exam_collect()
{
	unsigned result = 0;

	cout << "examining collect coefficients" << flush;

	result += exam_collect_1();  cout << '.' << flush;
	result += exam_collect_2();  cout << '.' << flush;
	result += exam_collect_3();  cout << '.' << flush;

	return result;
}

int main(int argc, char** argv)
{
	return exam_collect();
}
