#include <cln/number.h>
#include <cln/io.h>
#include <cln/integer.h>
#include <cln/random.h>
#include <cstdlib>
#include <cstring>
#include <cln/timing.h>

int main (int argc, char * argv[])
{
	int repetitions = 1;
	if ((argc >= 3) && !strcmp(argv[1],"-r")) {
		repetitions = atoi(argv[2]);
		argc -= 2; argv += 2;
	}
	if (argc < 2)
		exit(1);
	cl_I m1 = cl_I(argv[1]);
	cl_I M1 = (cl_I)1 << (intDsize*m1);
	cl_I m2 = (argc>2 ? cl_I(argv[2]) : m1);
	cl_I M2 = (cl_I)1 << (intDsize*m2);
	cl_I a = random_I(M1);
	cl_I b = random_I(M2);
	extern int cl_mul_algo;
	cl_mul_algo = 0;
	{ CL_TIMING;
	  for (int rep = repetitions; rep > 0; rep--)
	    { cl_I p = a * b; }
	}
	cl_mul_algo = 1;
	{ CL_TIMING;
	  for (int rep = repetitions; rep > 0; rep--)
	    { cl_I p = a * b; }
	}
	cl_mul_algo = 2;
	{ CL_TIMING;
	  for (int rep = repetitions; rep > 0; rep--)
	    { cl_I p = a * b; }
	}
	cl_mul_algo = 3;
	{ CL_TIMING;
	  for (int rep = repetitions; rep > 0; rep--)
	    { cl_I p = a * b; }
	}
	cl_mul_algo = 4;
	{ CL_TIMING;
	  for (int rep = repetitions; rep > 0; rep--)
	    { cl_I p = a * b; }
	}
}
