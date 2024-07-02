/** @file exam_pgcd.cpp
 *
 * Exam GCD over prime fields computations.
 */
#include <string>
#include <iostream>
#include <utility>
#include "ginac.h"
using namespace std;
using namespace GiNaC;

// Check for an infite loop in PGCD, fixed 2010-02-23.
static unsigned pgcd_relatively_prime_bug()
{
	const symbol q("q");
	parser reader;
	reader.get_syms().insert(make_pair(string("q"), q));

	ex t = reader("-E20^16*E4^8*E5^8*E1^16*q^4"
		      "-(E10^24-E20^8*E5^16)*E4^16*E1^8"
		      "+E2^24*E20^16*E5^8*q^4");
	ex g = gcd(t.expand(), t.diff(q).expand()) - 1;
	if (!g.is_zero()) {
		clog << " oops!" << endl <<
			"** Error: should be 0, got " << g << endl;
		return 1;
	}
	return 0;
}

// Check for an infinite loop in PGCD, fixed 2010-03-18.
static unsigned pgcd_infinite_loop()
{
	parser the_parser;
	ex e = the_parser(string(R"ex(792*z^8*w^4*x^3*y^4*u^7
+ 24*z^4*w^4*x^2*y^3*u^4 + 264*z^8*w^3*x^2*y^7*u^5 + 198*z^4*w^5*x^5*y*u^6
+ 110*z^2*w^3*x^5*y^4*u^6 - 120*z^8*w*x^4*u^6 - 480*z^5*w*x^4*y^6*u^8
- 720*z^7*x^3*y^3*u^7 + 165*z^4*w^2*x^4*y*u^5 + 450*z^8*w^6*x^2*y*u^8
+ 40*z^2*w^3*x^3*y^3*u^6 - 288*z^7*w^2*x^3*y^6*u^6 + 250*z^6*w^4*x^2*y^4*u^8
+ 576*z^7*w^7*x^2*y^4*u^8 - 80*z^6*w^2*x^5*y^3*u^7 - 144*z^8*w^4*x^5*u^7
+ 120*z^4*w*x^2*y^6*u^6 + 320*z^5*w^5*x^2*y^7*u^8 + 192*z^7*w^6*x*y^7*u^6
- 12*z^4*w^3*x^3*y^5*u^6 - 36*z^4*w^4*x^4*y^2*u^8 + 72*z^4*w^5*x^3*u^6
- 20*z^2*w^2*x^4*y^5*u^8 + 660*z^8*w*x^2*y^4*u^6 + 66*z^4*w^4*x^4*y^4*u^4
+ 440*z^6*w^2*x^3*y^7*u^7 - 30*z^4*w*x^3*y^2*u^7 - 48*z^8*w^3*x^4*y^3*u^5
+ 72*z^6*w^2*x*y^6*u^4 - 864*z^7*w^3*x^4*y^3*u^8 + 480*z^7*w^4*x*y^4*u^7
+ 60*z^4*w^2*x^2*u^5 + 375*z^8*w^3*x*y*u^7 + 150*z^8*w^5*x*y^4*u^6
+ 180*z^6*x*y^3*u^5 + 216*z^6*w^3*x^2*y^3*u^6)ex"));
	const symbol x = ex_to<symbol>(the_parser.get_syms()["x"]);
	ex g = gcd(e, e.diff(x));
	ex should_be = the_parser(string("u^4*z^2"));
	if (!(g-should_be).expand().is_zero()) {
		clog << "GCD was miscomputed. " << endl;
		return 1;
	}
	return 0;
}

int main()
{
	unsigned result = 0;

	cout << "Examining pgcd() bugs (infinite loop, miscalculation)" << flush;

	result += pgcd_relatively_prime_bug();  cout << '.' << flush;
	result += pgcd_infinite_loop();  cout << '.' << flush;

	return result;
}
