/** @file exam_relational.cpp
 *
 *  Small test for the relational objects and their conversion to bool. */

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

// Elementary relations should fall back to numeric comparisons.
static unsigned exam_relational_elementary()
{
	unsigned result = 0;
	ex one = 1, two = 2;

	if (one == two) {
		clog << "'" << one << " == " << two << "' was converted to true." << endl;
		result += 1;
	}
	if (!(one != two)) {
		clog << "'" << one << " != " << two << "' was not converted to true." << endl;
		result += 1;
	}
	if (!(one < two)) {
		clog << "'" << one << " < " << two << "' was not converted to true." << endl;
		result += 1;
	}
	if (!(one <= two)) {
		clog << "'" << one << " <= " << two << "' was not converted to true." << endl;
		result += 1;
	}
	if (one > two) {
		clog << "'" << one << " > " << two << "' was converted to true." << endl;
		result += 1;
	}
	if (one >= two) {
		clog << "'" << one << " >= " << two << "' was converted to true." << endl;
		result += 1;
	}

	return result;
}

// These should fall back to looking up info flags.
static unsigned exam_relational_possymbol()
{
	unsigned result = 0;
	possymbol p("p");

	if (p == 0) {
		clog << "for positive p, 'p == 0' was converted to true." << endl;
		result += 1;
	}
	if (!(p != 0)) {
		clog << "for positive p, 'p != 0' was not converted to true." << endl;
		result += 1;
	}
	if (p < 0) {
		clog << "for positive p, 'p < 0' was converted to true." << endl;
		result += 1;
	}
	if (p <= 0) {
		clog << "for positive p, 'p <= 0' was converted to true." << endl;
		result += 1;
	}
	if (!(p > 0)) {
		clog << "for positive p, 'p > 0' was not converted to true." << endl;
		result += 1;
	}
	if (!(p >= 0)) {
		clog << "for positive p, 'p >= 0' was not converted to true." << endl;
		result += 1;
	}

	return result;
}

// Very simple arithmetic should be supported, too.
static unsigned exam_relational_arith()
{
	unsigned result = 0;
	possymbol p("p");

	if (!(p + 2 > p + 1)) {
		clog << "for positive p, 'p + 2 > p + 1' was not converted to true." << endl;
		result += 1;
	}
	if (!(p > -p)) {
		clog << "for positive p, 'p > -p' was not converted to true." << endl;
		result += 1;
	}
	if (!(2*p > p)) {
		clog << "for positive p, '2*p > p' was not converted to true." << endl;
		result += 1;
	}

	return result;
}

// Comparisons should maintain ordering invariants
static unsigned exam_relational_order()
{
	unsigned result = 0;
	numeric i = 1ll<<32, j = i+1;
	symbol a;
	relational x = i==a, y = j==a;
	if (x.compare(y) != -y.compare(x)) {
		clog << "comparison should be antisymmetric." << endl;
		result += 1;
	}

	return result;
}

unsigned exam_relational()
{
	unsigned result = 0;

	cout << "examining relational objects" << flush;

	result += exam_relational_elementary(); cout << '.' << flush;
	result += exam_relational_possymbol(); cout << '.' << flush;
	result += exam_relational_arith(); cout << '.' << flush;
	result += exam_relational_order(); cout << '.' << flush;

	return result;
}

int main(int argc, char** argv)
{
	return exam_relational();
}
