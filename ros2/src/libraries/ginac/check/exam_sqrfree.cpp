/** @file exam_sqrfree.cpp
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

static unsigned exam_sqrfree1()
{
	unsigned result = 0;
	symbol x("x");
	ex e1, e2;

	e1 = (1+x)*pow((2+x),2)*pow((3+x),3)*pow((4+x),4);
	e2 = sqrfree(expand(e1), lst{x});
	if (e1 != e2) {
		clog << "sqrfree(expand(" << e1 << ")) erroneously returned "
		     << e2 << endl;
		++result;
	}

	return result;
}

static unsigned exam_sqrfree2()
{
	unsigned result = 0;
	symbol x("x"), y("y");
	ex e1, e2;

	e1 = (x+y)*pow((x+2*y),2)*pow((x+3*y),3)*pow((x+4*y),4);
	e2 = sqrfree(expand(e1));
	if (e1 != e2) {
		clog << "sqrfree(expand(" << e1 << ")) erroneously returned "
		     << e2 << endl;
		++result;
	}
	e2 = sqrfree(expand(e1), lst{x});
	if (e1 != e2) {
		clog << "sqrfree(expand(" << e1 << "),[x]) erroneously returned "
		     << e2 << endl;
		++result;
	}
	e2 = sqrfree(expand(e1), lst{y});
	if (e1 != e2) {
		clog << "sqrfree(expand(" << e1 << "),[y]) erroneously returned "
		     << e2 << endl;
		++result;
	}
	e2 = sqrfree(expand(e1), lst{x,y});
	if (e1 != e2) {
		clog << "sqrfree(expand(" << e1 << "),[x,y]) erroneously returned "
		     << e2 << endl;
		++result;
	}

	return result;
}

static unsigned exam_sqrfree3()
{
	unsigned result = 0;
	symbol x("x"), y("y"), z("z");
	ex e1, e2;

	e1 = (x+y)*pow(x, 2)*(-z-1);
	e2 = sqrfree(expand(e1));
	if (!expand(e1 - e2).is_zero()) {
		clog << "sqrfree(expand(" << e1 << ")) erroneously returned "
		     << e2 << endl;
		++result;
	}

	e1 = (x+y)*pow(x, 3)*(-z-1);
	e2 = sqrfree(expand(e1));
	if (!expand(e1 - e2).is_zero()) {
		clog << "sqrfree(expand(" << e1 << ")) erroneously returned "
		     << e2 << endl;
		++result;
	}

	return result;
}

// Bug in sqrfree_yun (fixed 2016-02-02).
static unsigned exam_hidden_zero1()
{
	unsigned result = 0;
	symbol x("x");
	ex e;

	e = (x-1)*(x+1) - x*x + 1;  // an unexpanded 0...
	try {
                ex f = sqrfree(e);
                if (!f.is_zero()) {
                        clog << "sqrfree(" << e << ") returns " << f << " instead of 0\n";
                        ++result;
                }
        } catch (const exception &err) {
                clog << "sqrfree(" << e << ") throws " << err.what() << endl;
                ++result;
        }

	e = pow(x-1,3) - expand(pow(x-1,3));  // ...still after differentiating...
	try {
                ex f = sqrfree(e);
                if (!f.is_zero()) {
                        clog << "sqrfree(" << e << ") returns " << f << " instead of 0\n";
                        ++result;
                }
        } catch (const exception &err) {
                clog << "sqrfree(" << e << ") throws " << err.what() << endl;
                ++result;
        }

	e = pow(x-1,4) - expand(pow(x-1,4));  // ...and after differentiating twice.
	try {
                ex f = sqrfree(e);
                if (!f.is_zero()) {
                        clog << "sqrfree(" << e << ") returns " << f << " instead of 0\n";
                        ++result;
                }
        } catch (const exception &err) {
                clog << "sqrfree(" << e << ") throws " << err.what() << endl;
                ++result;
        }

	return result;
}

static unsigned exam_hidden_zero2()
{
	unsigned result = 0;
	symbol x("x"), y("y");
	ex e1, e2;

	e1 = (1 + 3*x + 3*pow(x,2) + pow(x,3) - pow(1+x,3)) * y;
	e2 = sqrfree(e1);
	if (!e2.is_zero()) {
		clog << "sqrfree(" << e1 << ") erroneously returned "
		     << e2 << endl;
		++result;
	}

	e1 = (pow(x,2)-2*x*y+pow(y,2)-pow(x-y,2)) * x;
	e2 = sqrfree(e1);
	if (!e2.is_zero()) {
		clog << "sqrfree(" << e1 << ") erroneously returned "
		     << e2 << endl;
		++result;
	}

	e1 = (pow(x,2)-2*x*y+pow(y,2)-pow(x-y,2)) * (x+y);
	e2 = sqrfree(e1);
	if (!e2.is_zero()) {
		clog << "sqrfree(" << e1 << ") erroneously returned "
		     << e2 << endl;
		++result;
	}

	return result;
}

unsigned exam_sqrfree()
{
	unsigned result = 0;

	cout << "examining square-free factorization" << flush;

	result += exam_sqrfree1();  cout << '.' << flush;
	result += exam_sqrfree2();  cout << '.' << flush;
	result += exam_sqrfree3();  cout << '.' << flush;
	result += exam_hidden_zero1();  cout << '.' << flush;
	result += exam_hidden_zero2();  cout << '.' << flush;

	return result;
}

unsigned exam_sqrfree_parfrac()
{
	symbol x("x");
	// (expression, number of terms after partial fraction decomposition)
	vector<pair<ex, unsigned>> exams = {
		{ex("(x - 1) / (x^2*(x^2 + 2))", lst{x}), 3},
		{ex("(1 - x^10) / x", lst{x}), 2},
		{ex("(2*x^3 + x + 3) / ((x^2 + 1)^2)", lst{x}), 2},
		{ex("1 / (x * (x+1)^2 * (x+2)^3)", lst{x}), 6},
		{ex("(x*x + 3*x - 1) / (x^2*(x^2 + 2)^3)", lst{x}), 5},
		{ex("(1 - x^10) / (x + 2)", lst{x}), 11},
		{ex("(1 - x + 3*x^2) / (x^3 * (2+x)^2)", lst{x}), 5},
		{ex("(1 - x) / (x^4 * (x - 2)^3)", lst{x}), 6},
		{ex("(1 - 2*x + x^9) / (x^5 * (1 - x + x^2)^6)", lst{x}), 11}
	};
	unsigned result = 0;

	cout << "\n"
	     << "examining square-free partial fraction decomposition" << flush;
	for (auto e: exams) {
		ex e1 = e.first;
		ex e2 = sqrfree_parfrac(e1, x);
		if (e2.nops() != e.second ||
		    !is_a<add>(e2) ||
		    !normal(e1-e2).is_zero()) {
			clog << "sqrfree_parfrac(" << e1 << ", " << x << ") erroneously returned "
			     << e2 << endl;
			++result;
		}
		cout << '.' << flush;
	}

	return result;
}

int main(int argc, char** argv)
{
	unsigned result = 0;

	result += exam_sqrfree();
	result += exam_sqrfree_parfrac();

	return result;
}
