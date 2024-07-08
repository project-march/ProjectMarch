// round2().

// General includes.
#include "base/cl_sysdep.h"

// Specification.
#include "cln/float.h"


// Implementation.

#include "float/cl_F.h"

namespace cln {

const cl_F_div_t round2 (const cl_F& x, const cl_F& y)
{
// Methode:
// (q,r) := round(x/y). Liefere q und x-y*q = y*r.
	var cl_F_div_t q_r = round2(x/y);
	var cl_I& q = q_r.quotient;
	var cl_F& r = q_r.remainder;
	return cl_F_div_t(q,y*r);
}

}  // namespace cln
