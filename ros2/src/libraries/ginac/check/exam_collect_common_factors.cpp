#include <iostream>
#include "ginac.h"
using namespace std;
using namespace GiNaC;

static unsigned exam_collect_common_factors_simple()
{
	unsigned result = 0;
	symbol a("a"), b("b"), c("c"), x("x"), y("y");
	ex ei, ef, ref;

	ei = a*x + a*y;
	ef = collect_common_factors(ei);
	ref = (x+y)*a;
	if (ef != ref) {
		clog << "ERROR: collect_common_factors(" << ei << ") returned "
		     << ef << ", not " << ref << '.' << endl;
		++result;
	}

	ei = a*x*x + 2*a*x*y + a*y*y;
	ef = collect_common_factors(ei);
	ref = a*(x*x + 2*x*y + y*y);
	if (ef != ref) {
		clog << "ERROR: collect_common_factors(" << ei << ") returned "
		     << ef << ", not " << ref << '.' << endl;
		++result;
	}

	return result;
}

static unsigned exam_collect_common_factors_zero()
{
	// r = 0 = c*0 = c*(x + 1 - 1 - x) = c*(x + 1) - c - c*x
	// e = a*r - b*r
	symbol a("a"), b("b"), c("c"), x("x");

	ex r = c*(x+1) - c - c*x;
	ex ei = a*r + b*r;
	ex ef = collect_common_factors(ei);
	if (!ef.is_zero()) {
		clog << "ERROR: " << ei << " should be 0, got " << ef << " instead." << endl;
		return 1;
	}
	return 0;
}

int main(int argc, char** argv)
{
	int result = 0;

	cout << "examining collect_common_factors" << flush;

	result += exam_collect_common_factors_simple();  cout << '.' << flush;
	result += exam_collect_common_factors_zero();  cout << '.' << flush;

	return result;
}
