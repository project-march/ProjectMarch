#ifdef IN_GINAC
#include "ginac.h"
#else
#include "ginac/ginac.h"
#endif

#include <vector>
#include <iostream>

using namespace GiNaC;
using namespace std;

DECLARE_FUNCTION_2P(foobar);

static bool eval_called = false;
static exvector eval_called_with = {};
static bool evalf_called = false;
static exvector evalf_called_with = {};

static void reset() {
	eval_called_with.clear();
	evalf_called_with.clear();
	evalf_called = false;
	eval_called = false;
}

static ex foobar_eval(const exvector& args) {
	eval_called = true;
	for (auto const& v: args)
		eval_called_with.push_back(v);

	return foobar(args[0], args[1]).hold();
}

static ex foobar_evalf(const exvector& args) {
	evalf_called = true;
	for (auto const& v: args)
		evalf_called_with.push_back(v);
	return foobar(args[0], args[1]).hold();
}


REGISTER_FUNCTION(foobar, eval_func(foobar_eval).
			  evalf_func(foobar_evalf));

static int check_exvector_eval() {
	symbol x("x"), y("y");
	int err = 1;

	reset();
	ex e = foobar(x, y);
	if (!eval_called) {
		clog << "*** Error: " << __func__ << ": foobar_eval hasn't been called" << endl;
		err *= 2;
	}
	if (eval_called_with.size() != 2) {
		clog << "*** Error: " << __func__ << ": fobar_eval: expected 2 arguments, got " <<
			eval_called_with.size() << endl;
		err *= 3;
	}
	if (eval_called_with[0] != x) {
		clog << "*** Error: " << __func__ << ": fobar_eval: wrong 1st argument, "
			"expected " << x << ", got " << eval_called_with[0] << endl;
		err *= 5;
	}
	if (eval_called_with[1] != y) {
		clog << "*** Error: " << __func__ << ": fobar_eval: wrong 1st argument, "
			"expected " << y << ", got " << eval_called_with[1] << endl;
		err *= 7;
	}
	return err - 1;
}

static int check_exvector_evalf() {
	int err = 1;

	reset();
	ex e = foobar(Pi, Euler);
	e = e.evalf();

	if (!evalf_called) {
		clog << "*** Error: " << __func__ << ": foobar_evalf hasn't been called" << endl;
		err *= 2;
	}
	if (evalf_called_with.size() != 2) {
		clog << __func__ << ": foobar_evalf: expected 2 arguments, got " <<
			evalf_called_with.size() << endl;
		err *= 3;
	}
	if (!is_a<numeric>(evalf_called_with[0])) {
		clog << "*** Error: " << __func__ << ": wrong 1st argument of foobar_evalf: "
			"expected a real number, got " << evalf_called_with[0] << endl;
		err *= 5;
	}
	if (!is_a<numeric>(evalf_called_with[1])) {
		clog << "*** Error: " << __func__ << ": wrong 1st argument of foobar_evalf: "
			"expected a real number, got " << evalf_called_with[0] << endl;
		err *= 7;
	}
	return err - 1;
}

int main(int argc, char** argv) {
	int ret = 0;
	auto err = check_exvector_evalf();
	if (err) {
		ret |= 1;
		clog << "*** Error " << (err + 1) << " (check_exvector_evalf)" << endl;
	}
	err = check_exvector_eval();
	if (err) { 
		ret |= 2;
		clog << "*** Error " << (err + 1) << " (check_exvector_evalf)" << endl;
	}
	return ret;
}
