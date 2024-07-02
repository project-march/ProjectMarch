// Fast integer multiplication using FFT over the complex numbers.
// [Donald Ervin Knuth: The Art of Computer Programming, Vol. II:
//  Seminumerical Algorithms, second edition. Section 4.3.3, p. 290-294.]
// Bruno Haible 6.5.1996, 24.-25.8.1996

// FFT in the complex domain has the drawback that it needs careful round-off
// error analysis. But for CPUs with good floating-point performance it might
// nevertheless be better than FFT mod Z/mZ.

// It is important to have precise round-off error estimates. Although
// (by laws of statistics) in the average the actual round-off error makes up
// only half of the bits provided for round-off protection, we cannot rely
// on this average behaviour, but have to produce correct results.
//
// Knuth's formula (42), p. 294, says:
// If we want to multiply l-bit words using an FFT(2^k), our floating point
// numbers shall have m >= 2(k+l) + k + log_2 k + 3.5 mantissa bits.
//
// Here is a more careful analysis, using absolute error estimates.
//
// 1. We assume floating point numbers with radix 2, with the properties:
//    (i)   Multiplication with 2^n and 2^-n is exact.
//    (ii)  Negation x -> -x is exact.
//    (iii) Addition: When adding x and y, with |x| <= 2^a, |y| <= 2^a,
//          the result |x+y| <= 2^(a+1) has an error <= e*2^(a+1-m).
//    (iv)  Multiplication: When multiplying x and y, with |x| <= 2^a,
//          |y| <= 2^b, the result |x*y| <= 2^(a+b) has an error <= e*2^(a+b-m).
//    Here e = 1 for a truncating arithmetic, but e = 1/2 for a rounding
//    arithmetic like IEEE single and double floats.
// 2. Let's introduce some notation: err(x) means |x'-x| where x is the
//    exact mathematical value and x' is its representation in the machine.
// 3. From 1. we get for real numbers x,y:
//    (i)   err(2^n*x) = 2^n * err(x),
//    (ii)  err(-x) = err(x),
//    (iii) |x| <= 2^a, |y| <= 2^a, then
//          err(x+y) <= err(x) + err(y) + e*2^(a+1-m),
//          [or ....    ............... + e*2^(a-m) if |x+y| <= 2^a].
//    (iv)  |x| <= 2^a, |y| <= 2^b, then
//          err(x*y) <= 2^a * err(y) + 2^b * err(x) + e*2^(a+b-m).
// 4. Our complex arithmetic will be based on the formulas:
//    (i)   2^n*(x+iy) = (2^n*x)+i(2^n*y)
//    (ii)  -(x+iy) = (-x)+i(-y)
//    (iii) (x+iy)+(u+iv) = (x+u)+i(y+v)
//    (iv)  (x+iy)*(u+iv) = (x*u-y*v)+i(x*v+y*u)
//    The notation err(z) means |z'-z|, as above, with |.| being the usual
//    absolute value on complex numbers (_not_ the L^1 norm).
// 5. From 3. and 4. we get for complex numbers x,y:
//    (i)   err(2^n*x) = 2^n * err(x),
//    (ii)  err(-x) = err(x),
//    (iii) |x| <= 2^a, |y| <= 2^a, then
//          err(x+y) <= err(x) + err(y) + e*2^(a+3/2-m),
//    (iv)  |x| <= 2^a, |y| <= 2^b, then
//          err(x*y) <= 2^a * err(y) + 2^b * err(x) + 3*e*2^(a+b+1/2-m).
// 6. We start out with precomputed roots of unity:
//          |exp(2 pi i/2^n)| <= 1,
//          err(exp(2 pi i/2^n)) <= e*2^(1/2-m), (even err(..)=0 for n=0,1,2),
//    and compute  exp(2 pi i * j/2^k)  according to the binary digits of j.
//    This way, each root of unity will be a product of at most k precomputed
//    roots. If (j mod 2^(k-2)) has n bits, then  exp(2 pi i * j/2^k)  will
//    be computed using n factors, i.e. n-1 complex multiplications, and by
//    5.iv. we'll have
//          err(exp(2 pi i * j/2^k)) <= n*e*2^(1/2-m) + max(n-1,0)*3*e*2^(1/2-m)
//                                    = max(4*n-3,0)*e*2^(1/2-m).
//    Hence the maximum roots-of-unity error is (set n=k-2)
//          err(w^j) <= (4*k-11)*e*2^(1/2-m),
//    and the average roots-of-unity error is (set n=(k-2)/2)
//          < 2*(k-2)*e*2^(1/2-m).
// 7. Now we start the FFT.
//    Before the first step, x_i are integral, |x_i| < 2^l and err(x_i) = 0.
//    After the first butterfly, which replaces (x(i1),x(i2)) by
//    (x(i1) + x(i2), x(i1) - x(i2)), we have |x_i| < 2^(l+1) and err(x_i) = 0.
//    Then, for each of the remaining k-1 steps, a butterfly replaces
//    (x(i1),x(i2)) by (x(i1) + w^exp*x(i2), x(i1) - w^exp*x(i2)). Thus,
//    after n steps we have |x_i| < 2^(l+n) and err(x_i) <= E_i where
//          E_0 = 0,
//          E_1 = 0,
//          E_(n+1) = (E_n + 2^(l+n)*(4*k-11)*e*2^(1/2-m) + 3*e*2^(l+n+1/2-m))
//                    + E_n + e*2^(l+n+3/2-m)
//                  = 2*E_n + (4*k-6)*e*2^(l+n+1/2-m)
//    hence  E_n = (2^n-2)*(4*k-6)*e*2^(l+1/2-m).
//    Setting n = k, we have proved that after the FFT ends, we have
//          |x_i| < 2^(l+k) and err(x_i) <= (4*k-6)*e*2^(l+k+1/2-m).
// 8. The same error analysis holds for the y_i and their FFT. After we
//    multiply z_i := x_i * y_i, we have
//          |z_i| < 2^(2*l+2*k) and err(z_i) <= (8*k-9)*e*2^(2*l+2*k+1/2-m).
// 9. Then an inverse FFT on z_i is done, which is the same as an FFT
//    followed by a permutation and a division by 2^k. After n steps of
//    the FFT, we have |z_i| < 2^(2*l+2*k+n) and err(z_i) <= E_i where
//          E_0 = (8*k-9)*e*2^(2*l+2*k+1/2-m),
//          E_(n+1) = (E_n + 2^(2*l+2*k+n)*(4*k-11)*e*2^(1/2-m)
//                         + 3*e*2^(2*l+2*k+n+1/2-m))
//                    + E_n + e*2^(2*l+2*k+n+3/2-m)
//                  = 2*E_n + (4*k-6)*e*2^(2*l+2*k+n+1/2-m)
//    hence  E_n = 2^n*(8*k-9)*e*2^(2*l+2*k+1/2-m)
//                 + (2^n-1)*(4*k-6)*e*2^(2*l+2*k+1/2-m).
//    So, after the FFT, we have (set n=k) |z_i| < 2^(2*l+3*k) and
//          err(z_i) <= (12*k-15)*e*2^(2*l+3*k+1/2-m).
//    Permutation doesn't change the estimates. After division by 2^k, we get
//    |z_i| < 2^(2*l+2*k) and
//          err(z_i) <= (12*k-15)*e*2^(2*l+2*k+1/2-m).
// 10. When converting the z_i back to integers, we know that z_i should be
//     real, integral, and |z_i| < 2^(2*l+k). We can only guarantee that we
//     can find the integral z_i from the floating-point computation if
//          (12*k-15)*e*2^(2*l+2*k+1/2-m) < 1/2.
// 11. Assuming e = 1/2 and m = 53 (typical values for IEEE double arithmetic),
//     we get the constraint  2*l < m - 1/2 - 2*k - log_2(12*k-15).
//          k = 2    l <= 22
//          k = 3    l <= 21
//          k = 4    l <= 19
//          k = 5    l <= 18
//          k = 6    l <= 17
//          k = 7    l <= 16
//          k = 8    l <= 15
//          k = 9    l <= 13
//          k = 10   l <= 12
//          k = 11   l <= 11
//          k = 12   l <= 10
//          k = 13   l <= 9
//          k = 14   l <= 8
//          k = 15   l <= 7
//          k = 16   l <= 6
//          k = 17   l <= 5
//          k = 18   l <= 4
//          k = 19   l <= 3
//          k = 20   l <= 2
//     Assuming e = 1/2 and m = 64 ("long double" arithmetic on i387/i486/i586),
//     we get the constraint  2*l < m - 1/2 - 2*k - log_2(12*k-15).
//          k = 2    l <= 28
//          k = 3    l <= 26
//          k = 4    l <= 25
//          k = 5    l <= 24
//          k = 6    l <= 22
//          k = 7    l <= 21
//          k = 8    l <= 20
//          k = 9    l <= 19
//          k = 10   l <= 18
//          k = 11   l <= 17
//          k = 12   l <= 16
//          k = 13   l <= 15
//          k = 14   l <= 14
//          k = 15   l <= 13
//          k = 16   l <= 12
//          k = 17   l <= 10
//          k = 18   l <= 9
//          k = 19   l <= 8
//          k = 20   l <= 7
//          k = 21   l <= 6
//          k = 22   l <= 5
//          k = 23   l <= 4
//          k = 24   l <= 3
//          k = 25   l <= 2


#if !(intDsize==32)
#error "complex fft implemented only for intDsize==32"
#endif


#include "cln/floatparam.h"
#include "cln/exception.h"

#if (long_double_mant_bits > double_mant_bits) && (defined(__i386__) || defined(__m68k__) || (defined(__sparc__) && 0))
// Only these CPUs have fast "long double"s in hardware.
// On SPARC, "long double"s are emulated in software and don't work.
typedef long double fftc_real;
#define fftc_real_mant_bits long_double_mant_bits
#define fftc_real_rounds long_double_rounds
#else
typedef double fftc_real;
#define fftc_real_mant_bits double_mant_bits
#define fftc_real_rounds double_rounds
#endif

typedef struct fftc_complex
  {
    fftc_real re;
    fftc_real im;
  }
  fftc_complex;

static const fftc_complex fftc_roots_of_1 [32+1] =
  // roots_of_1[n] is a (2^n)th root of unity in C.
  // Also roots_of_1[n-1] = roots_of_1[n]^2.
  // For simplicity we choose  roots_of_1[n] = exp(2 pi i/2^n).
  {
   #if (fftc_real_mant_bits == double_mant_bits)
    // These values have 64 bit precision.
    { 1.0,                    0.0 },
    { -1.0,                   0.0 },
    { 0.0,                    1.0 },
    { 0.7071067811865475244,  0.7071067811865475244 },
    { 0.9238795325112867561,  0.38268343236508977172 },
    { 0.9807852804032304491,  0.19509032201612826784 },
    { 0.99518472667219688623, 0.098017140329560601996 },
    { 0.9987954562051723927,  0.049067674327418014254 },
    { 0.9996988186962042201,  0.024541228522912288032 },
    { 0.99992470183914454094, 0.0122715382857199260795 },
    { 0.99998117528260114264, 0.0061358846491544753597 },
    { 0.9999952938095761715,  0.00306795676296597627 },
    { 0.99999882345170190993, 0.0015339801862847656123 },
    { 0.99999970586288221914, 7.6699031874270452695e-4 },
    { 0.99999992646571785114, 3.8349518757139558907e-4 },
    { 0.9999999816164292938,  1.9174759731070330744e-4 },
    { 0.9999999954041073129,  9.5873799095977345874e-5 },
    { 0.99999999885102682754, 4.793689960306688455e-5 },
    { 0.99999999971275670683, 2.3968449808418218729e-5 },
    { 0.9999999999281891767,  1.1984224905069706422e-5 },
    { 0.99999999998204729416, 5.9921124526424278428e-6 },
    { 0.99999999999551182357, 2.9960562263346607504e-6 },
    { 0.99999999999887795586, 1.4980281131690112288e-6 },
    { 0.999999999999719489,   7.4901405658471572114e-7 },
    { 0.99999999999992987223, 3.7450702829238412391e-7 },
    { 0.99999999999998246807, 1.8725351414619534487e-7 },
    { 0.999999999999995617,   9.36267570730980828e-8 },
    { 0.99999999999999890425, 4.6813378536549092695e-8 },
    { 0.9999999999999997261,  2.340668926827455276e-8 },
    { 0.99999999999999993153, 1.1703344634137277181e-8 },
    { 0.99999999999999998287, 5.8516723170686386908e-9 },
    { 0.9999999999999999957,  2.925836158534319358e-9 },
    { 0.9999999999999999989,  1.4629180792671596806e-9 }
   #else (fftc_real_mant_bits > double_mant_bits)
    // These values have 128 bit precision.
    { 1.0L,                                       0.0L },
    { -1.0L,                                      0.0L },
    { 0.0L,                                       1.0L },
    { 0.707106781186547524400844362104849039284L, 0.707106781186547524400844362104849039284L },
    { 0.923879532511286756128183189396788286823L, 0.38268343236508977172845998403039886676L },
    { 0.980785280403230449126182236134239036975L, 0.195090322016128267848284868477022240928L },
    { 0.995184726672196886244836953109479921574L, 0.098017140329560601994195563888641845861L },
    { 0.998795456205172392714771604759100694444L, 0.0490676743274180142549549769426826583147L },
    { 0.99969881869620422011576564966617219685L,  0.0245412285229122880317345294592829250654L },
    { 0.99992470183914454092164649119638322435L,  0.01227153828571992607940826195100321214037L },
    { 0.999981175282601142656990437728567716173L, 0.00613588464915447535964023459037258091705L },
    { 0.999995293809576171511580125700119899554L, 0.00306795676296597627014536549091984251894L },
    { 0.99999882345170190992902571017152601905L,  0.001533980186284765612303697150264079079954L },
    { 0.999999705862882219160228217738765677117L, 7.66990318742704526938568357948576643142e-4L },
    { 0.99999992646571785114473148070738785695L,  3.83495187571395589072461681181381263396e-4L },
    { 0.999999981616429293808346915402909714504L, 1.91747597310703307439909561989000933469e-4L },
    { 0.99999999540410731289097193313960614896L,  9.58737990959773458705172109764763511872e-5L },
    { 0.9999999988510268275626733077945541084L,   4.79368996030668845490039904946588727468e-5L },
    { 0.99999999971275670684941397221864177609L,  2.39684498084182187291865771650218200947e-5L },
    { 0.999999999928189176709775095883850490262L, 1.198422490506970642152156159698898480473e-5L },
    { 0.99999999998204729417728262414778410738L,  5.99211245264242784287971180889086172999e-6L },
    { 0.99999999999551182354431058417299732444L,  2.99605622633466075045481280835705981183e-6L },
    { 0.999999999998877955886077016551752536504L, 1.49802811316901122885427884615536112069e-6L },
    { 0.999999999999719488971519214794719584451L, 7.49014056584715721130498566730655637157e-7L },
    { 0.99999999999992987224287980123972873676L,  3.74507028292384123903169179084633177398e-7L },
    { 0.99999999999998246806071995015624773673L,  1.8725351414619534486882457659356361712e-7L },
    { 0.999999999999995617015179987529456656217L, 9.3626757073098082799067286680885620193e-8L },
    { 0.999999999999998904253794996881763834182L, 4.68133785365490926951155181385400969594e-8L },
    { 0.99999999999999972606344874922040343793L,  2.34066892682745527595054934190348440379e-8L },
    { 0.999999999999999931515862187305098514444L, 1.170334463413727718124621350323810379807e-8L },
    { 0.999999999999999982878965546826274482047L, 5.8516723170686386908097901008341396944e-9L },
    { 0.999999999999999995719741386706568611352L, 2.92583615853431935792823046906895590202e-9L },
    { 0.999999999999999998929935346676642152265L, 1.46291807926715968052953216186596371037e-9L }
   #endif
  };

// Define this for (cheap) consistency checks.
//#define DEBUG_FFTC

// Define the algorithm of the backward FFT:
// Either FORWARD (a normal FFT followed by a permutation)
// or     RECIPROOT (an FFT with reciprocal root of unity)
// or     CLEVER (an FFT with reciprocal root of unity but clever computation
//                of the reciprocals).
// Drawback of FORWARD: the permutation pass.
// Drawback of RECIPROOT: need all the powers of the root, not only half of them.
#define FORWARD   42
#define RECIPROOT 43
#define CLEVER    44
#define FFTC_BACKWARD CLEVER

static fftc_real fftc_pow2_table[64] = // table of powers of 2
  {
                      1.0,
                      2.0,
                      4.0,
                      8.0,
                     16.0,
                     32.0,
                     64.0,
                    128.0,
                    256.0,
                    512.0,
                   1024.0,
                   2048.0,
                   4096.0,
                   8192.0,
                  16384.0,
                  32768.0,
                  65536.0,
                 131072.0,
                 262144.0,
                 524288.0,
                1048576.0,
                2097152.0,
                4194304.0,
                8388608.0,
               16777216.0,
               33554432.0,
               67108864.0,
              134217728.0,
              268435456.0,
              536870912.0,
             1073741824.0,
             2147483648.0,
             4294967296.0,
             8589934592.0,
            17179869184.0,
            34359738368.0,
            68719476736.0,
           137438953472.0,
           274877906944.0,
           549755813888.0,
          1099511627776.0,
          2199023255552.0,
          4398046511104.0,
          8796093022208.0,
         17592186044416.0,
         35184372088832.0,
         70368744177664.0,
        140737488355328.0,
        281474976710656.0,
        562949953421312.0,
       1125899906842624.0,
       2251799813685248.0,
       4503599627370496.0,
       9007199254740992.0,
      18014398509481984.0,
      36028797018963968.0,
      72057594037927936.0,
     144115188075855872.0,
     288230376151711744.0,
     576460752303423488.0,
    1152921504606846976.0,
    2305843009213693952.0,
    4611686018427387904.0,
    9223372036854775808.0
  };

// For a constant expression n (0 <= n < 128), returns 2^n of type fftc_real.
#define fftc_pow2(n)  \
  (((n) & 64 ? (fftc_real)18446744073709551616.0 : (fftc_real)1.0)	\
   * ((n) & 32 ? (fftc_real)4294967296.0 : (fftc_real)1.0)		\
   * ((n) & 16 ? (fftc_real)65536.0 : (fftc_real)1.0)			\
   * ((n) & 8 ? (fftc_real)256.0 : (fftc_real)1.0)			\
   * ((n) & 4 ? (fftc_real)16.0 : (fftc_real)1.0)			\
   * ((n) & 2 ? (fftc_real)4.0 : (fftc_real)1.0)			\
   * ((n) & 1 ? (fftc_real)2.0 : (fftc_real)1.0)			\
  )

// r := a + b
static inline void add (const fftc_complex& a, const fftc_complex& b, fftc_complex& r)
{
	r.re = a.re + b.re;
	r.im = a.im + b.im;
}

// r := a - b
static inline void sub (const fftc_complex& a, const fftc_complex& b, fftc_complex& r)
{
	r.re = a.re - b.re;
	r.im = a.im - b.im;
}

// r := a * b
static inline void mul (fftc_real a, const fftc_complex& b, fftc_complex& r)
{
	r.re = a * b.re;
	r.im = a * b.im;
}

// r := a * b
static inline void mul (const fftc_complex& a, const fftc_complex& b, fftc_complex& r)
{
	var fftc_real r_re = a.re * b.re - a.im * b.im;
	var fftc_real r_im = a.re * b.im + a.im * b.re;
	r.re = r_re;
	r.im = r_im;
}

#ifndef _BIT_REVERSE
#define _BIT_REVERSE
// Reverse an n-bit number x. n>0.
static uintC bit_reverse (uintL n, uintC x)
{
	var uintC y = 0;
	do {
		y <<= 1;
		y |= (x & 1);
		x >>= 1;
	} while (!(--n == 0));
	return y;
}
#endif

// Compute a complex convolution using FFT: z[0..N-1] := x[0..N-1] * y[0..N-1].
static void fftc_convolution (const uintL n, const uintC N, // N = 2^n
                              fftc_complex * x, // N numbers
                              fftc_complex * y, // N numbers
                              fftc_complex * z  // N numbers result
                             )
{
	CL_ALLOCA_STACK;
	#if (FFTC_BACKWARD == RECIPROOT) || defined(DEBUG_FFTC)
	var fftc_complex* const w = cl_alloc_array(fftc_complex,N);
	#else
	var fftc_complex* const w = cl_alloc_array(fftc_complex,(N>>1)+1);
	#endif
	var uintC i;
	// Initialize w[i] to w^i, w a primitive N-th root of unity.
	w[0] = fftc_roots_of_1[0];
	#if (FFTC_BACKWARD == RECIPROOT) || defined(DEBUG_FFTC)
	{
		var int j;
		for (j = n-1; j>=0; j--) {
			var fftc_complex r_j = fftc_roots_of_1[n-j];
			w[1<<j] = r_j;
			for (i = (2<<j); i < N; i += (2<<j))
				mul(w[i],r_j, w[i+(1<<j)]);
		}
	}
	#else
	{
		var int j;
		for (j = n-1; j>=0; j--) {
			var fftc_complex r_j = fftc_roots_of_1[n-j];
			w[1<<j] = r_j;
			for (i = (2<<j); i < N>>1; i += (2<<j))
				mul(w[i],r_j, w[i+(1<<j)]);
		}
	}
	#endif
	#ifdef DEBUG_FFTC
	// Check that w is really a primitive N-th root of unity.
	{
		var fftc_real epsilon;
		epsilon = (fftc_real)(n>=3 ? 4*n-11 : 0);
		epsilon = epsilon / fftc_pow2(fftc_real_mant_bits);
		if (fftc_real_rounds == rounds_to_nearest)
			epsilon = 0.5*epsilon;
		epsilon = 1.414*epsilon;
		// epsilon = (4*k-11)*e*2^(1/2-m).
		var fftc_real part;
		var fftc_complex& w_N = w[N>>1];
		part = w_N.re - (fftc_real)(-1.0);
		if (part > epsilon || part < -epsilon)
			throw runtime_exception();
		part = w_N.im;
		if (part > epsilon || part < -epsilon)
			throw runtime_exception();
	}
	{
		var fftc_complex w_N;
		mul(w[N-1],fftc_roots_of_1[n], w_N);
		// Since there was one more multiplication,
		// we have to replace n by (n+1) in the epsilon above.
		var fftc_real epsilon;
		epsilon = (fftc_real)(4*n-7);
		epsilon = epsilon / fftc_pow2(fftc_real_mant_bits);
		if (fftc_real_rounds == rounds_to_nearest)
			epsilon = 0.5*epsilon;
		epsilon = 1.414*epsilon;
		// epsilon = (4*k-7)*e*2^(1/2-m).
		var fftc_real part;
		part = w_N.re - (fftc_real)1.0;
		if (part > epsilon || part < -epsilon)
			throw runtime_exception();
		part = w_N.im;
		if (part > epsilon || part < -epsilon)
			throw runtime_exception();
	}
	#endif
	var bool squaring = (x == y);
	// Do an FFT of length N on x.
	{
		var sintL l;
		/* l = n-1 */ {
			var const uintC tmax = N>>1; // tmax = 2^(n-1)
			for (var uintC t = 0; t < tmax; t++) {
				var uintC i1 = t;
				var uintC i2 = i1 + tmax;
				// Butterfly: replace (x(i1),x(i2)) by
				// (x(i1) + x(i2), x(i1) - x(i2)).
				var fftc_complex tmp;
				tmp = x[i2];
				sub(x[i1],tmp, x[i2]);
				add(x[i1],tmp, x[i1]);
			}
		}
		for (l = n-2; l>=0; l--) {
			var const uintC smax = (uintC)1 << (n-1-l);
			var const uintC tmax = (uintC)1 << l;
			for (var uintC s = 0; s < smax; s++) {
				var uintC exp = bit_reverse(n-1-l,s) << l;
				for (var uintC t = 0; t < tmax; t++) {
					var uintC i1 = (s << (l+1)) + t;
					var uintC i2 = i1 + tmax;
					// Butterfly: replace (x(i1),x(i2)) by
					// (x(i1) + w^exp*x(i2), x(i1) - w^exp*x(i2)).
					var fftc_complex tmp;
					mul(x[i2],w[exp], tmp);
					sub(x[i1],tmp, x[i2]);
					add(x[i1],tmp, x[i1]);
				}
			}
		}
	}
	// Do an FFT of length N on y.
	if (!squaring) {
		var sintL l;
		/* l = n-1 */ {
			var uintC const tmax = N>>1; // tmax = 2^(n-1)
			for (var uintC t = 0; t < tmax; t++) {
				var uintC i1 = t;
				var uintC i2 = i1 + tmax;
				// Butterfly: replace (y(i1),y(i2)) by
				// (y(i1) + y(i2), y(i1) - y(i2)).
				var fftc_complex tmp;
				tmp = y[i2];
				sub(y[i1],tmp, y[i2]);
				add(y[i1],tmp, y[i1]);
			}
		}
		for (l = n-2; l>=0; l--) {
			var const uintC smax = (uintC)1 << (n-1-l);
			var const uintC tmax = (uintC)1 << l;
			for (var uintC s = 0; s < smax; s++) {
				var uintC exp = bit_reverse(n-1-l,s) << l;
				for (var uintC t = 0; t < tmax; t++) {
					var uintC i1 = (s << (l+1)) + t;
					var uintC i2 = i1 + tmax;
					// Butterfly: replace (y(i1),y(i2)) by
					// (y(i1) + w^exp*y(i2), y(i1) - w^exp*y(i2)).
					var fftc_complex tmp;
					mul(y[i2],w[exp], tmp);
					sub(y[i1],tmp, y[i2]);
					add(y[i1],tmp, y[i1]);
				}
			}
		}
	}
	// Multiply the transformed vectors into z.
	for (i = 0; i < N; i++)
		mul(x[i],y[i], z[i]);
	// Undo an FFT of length N on z.
	{
		var uintL l;
		for (l = 0; l < n-1; l++) {
			var const uintC smax = (uintC)1 << (n-1-l);
			var const uintC tmax = (uintC)1 << l;
			#if FFTC_BACKWARD != CLEVER
			for (var uintC s = 0; s < smax; s++) {
				var uintC exp = bit_reverse(n-1-l,s) << l;
				#if FFTC_BACKWARD == RECIPROOT
				if (exp > 0)
					exp = N - exp; // negate exp (use w^-1 instead of w)
				#endif
				for (var uintC t = 0; t < tmax; t++) {
					var uintC i1 = (s << (l+1)) + t;
					var uintC i2 = i1 + tmax;
					// Inverse Butterfly: replace (z(i1),z(i2)) by
					// ((z(i1)+z(i2))/2, (z(i1)-z(i2))/(2*w^exp)).
					// Do the division by 2 later.
					var fftc_complex sum;
					var fftc_complex diff;
					add(z[i1],z[i2], sum);
					sub(z[i1],z[i2], diff);
					z[i1] = sum;
					mul(diff,w[exp], z[i2]);
				}
			}
			#else // FFTC_BACKWARD == CLEVER: clever handling of negative exponents
			/* s = 0, exp = 0 */ {
				for (var uintC t = 0; t < tmax; t++) {
					var uintC i1 = t;
					var uintC i2 = i1 + tmax;
					// Inverse Butterfly: replace (z(i1),z(i2)) by
					// ((z(i1)+z(i2))/2, (z(i1)-z(i2))/(2*w^exp)),
					// with exp <-- 0.
					// Do the division by 2 later.
					var fftc_complex sum;
					var fftc_complex diff;
					add(z[i1],z[i2], sum);
					sub(z[i1],z[i2], diff);
					z[i1] = sum;
					z[i2] = diff;
				}
			}
			for (var uintC s = 1; s < smax; s++) {
				var uintC exp = bit_reverse(n-1-l,s) << l;
				exp = (N>>1) - exp; // negate exp (use w^-1 instead of w)
				for (var uintC t = 0; t < tmax; t++) {
					var uintC i1 = (s << (l+1)) + t;
					var uintC i2 = i1 + tmax;
					// Inverse Butterfly: replace (z(i1),z(i2)) by
					// ((z(i1)+z(i2))/2, (z(i1)-z(i2))/(2*w^exp)),
					// with exp <-- (N/2 - exp).
					// Do the division by 2 later.
					var fftc_complex sum;
					var fftc_complex diff;
					add(z[i1],z[i2], sum);
					sub(z[i2],z[i1], diff); // note that w^(N/2) = -1
					z[i1] = sum;
					mul(diff,w[exp], z[i2]);
				}
			}
			#endif
		}
		/* l = n-1 */ {
			var const fftc_real pow2 = (fftc_real)1 / (fftc_real)N;
			var const uintC tmax = N>>1; // tmax = 2^(n-1)
			for (var uintC t = 0; t < tmax; t++) {
				var uintC i1 = t;
				var uintC i2 = i1 + tmax;
				// Inverse Butterfly: replace (z(i1),z(i2)) by
				// ((z(i1)+z(i2))/2, (z(i1)-z(i2))/2).
				// Do all the divisions by 2 now.
				var fftc_complex sum;
				var fftc_complex diff;
				add(z[i1],z[i2], sum);
				sub(z[i1],z[i2], diff);
				mul(pow2,sum, z[i1]);
				mul(pow2,diff, z[i2]);
			}
		}
	}
	#if FFTC_BACKWARD == FORWARD
	// Swap z[i] and z[N-i] for 0 < i < N/2.
	for (i = (N>>1)-1; i > 0; i--) {
		var fftc_complex tmp = z[i];
		z[i] = z[N-i];
		z[N-i] = tmp;
	}
	#endif
}

// For a given k >= 2, the maximum l is determined by
// 2*l < m - 1/2 - 2*k - log_2(12*k-15) - (1 if e=1.0, 0 if e=0.5).
// This is a decreasing function of k.
#define max_l(k)  \
	(int)((fftc_real_mant_bits			\
	       - 2*(k)					\
	       - ((k)<=2 ? 4 : (k)<=3 ? 5 : (k)<=5 ? 6 : (k)<=8 ? 7 : (k)<=16 ? 8 : (k)<=31 ? 9 : 10) \
	       - (fftc_real_rounds == rounds_to_nearest ? 0 : 1)) \
	      / 2)
static int max_l_table[32+1] =
  {         0,         0,  max_l(2),  max_l(3),  max_l(4),  max_l(5),  max_l(6),
     max_l(7),  max_l(8),  max_l(9), max_l(10), max_l(11), max_l(12), max_l(13),
    max_l(14), max_l(15), max_l(16), max_l(17), max_l(18), max_l(19), max_l(20),
    max_l(21), max_l(22), max_l(23), max_l(24), max_l(25), max_l(26), max_l(27),
    max_l(28), max_l(29), max_l(30), max_l(31), max_l(32)
  };

// Split len uintD's below sourceptr into chunks of l bits, thus filling
// N complex numbers at x.
static void fill_factor (uintC N, fftc_complex* x, uintL l,
                         const uintD* sourceptr, uintC len)
{
	var uintC i;
	if (max_l(2) > intDsize && l > intDsize) {
		// l > intDsize
		if (max_l(2) > 64 && l > 64) {
			throw runtime_exception("FFT problem: l > 64 not supported by pow2_table");
		}
		var fftc_real carry = 0;
		var sintL carrybits = 0; // number of bits in carry (>=0, <l)
		i = 0;
		while (len > 0) {
			var uintD digit = lsprefnext(sourceptr);
			if (carrybits+intDsize >= l) {
				x[i].re = carry + (fftc_real)(digit & bitm(l-carrybits)) * fftc_pow2_table[carrybits];
				x[i].im = (fftc_real)0;
				i++;
				carry = (l-carrybits == intDsize ? (fftc_real)0 : (fftc_real)(digit >> (l-carrybits)));
				carrybits = carrybits+intDsize-l;
			} else {
				carry = carry + (fftc_real)digit * fftc_pow2_table[carrybits];
				carrybits = carrybits+intDsize;
			}
			len--;
		}
		if (carrybits > 0) {
			x[i].re = carry;
			x[i].im = (fftc_real)0;
			i++;
		}
		if (i > N)
			throw runtime_exception();
	} else if (max_l(2) >= intDsize && l == intDsize) {
		// l = intDsize
		if (len > N)
			throw runtime_exception();
		for (i = 0; i < len; i++) {
			var uintD digit = lsprefnext(sourceptr);
			x[i].re = (fftc_real)digit;
			x[i].im = (fftc_real)0;
		}
	} else {
		// l < intDsize
		var const uintD l_mask = bit(l)-1;
		var uintD carry = 0;
		var sintL carrybits = 0; // number of bits in carry (>=0, <intDsize)
		for (i = 0; i < N; i++) {
			if (carrybits >= l) {
				x[i].re = (fftc_real)(carry & l_mask);
				x[i].im = (fftc_real)0;
				carry >>= l;
				carrybits -= l;
			} else {
				if (len == 0)
					break;
				len--;
				var uintD digit = lsprefnext(sourceptr);
				x[i].re = (fftc_real)((carry | (digit << carrybits)) & l_mask);
				x[i].im = (fftc_real)0;
				carry = digit >> (l-carrybits);
				carrybits = intDsize - (l-carrybits);
			}
		}
		while (carrybits > 0) {
			if (!(i < N))
				throw runtime_exception();
			x[i].re = (fftc_real)(carry & l_mask);
			x[i].im = (fftc_real)0;
			carry >>= l;
			carrybits -= l;
			i++;
		}
		if (len > 0)
			throw runtime_exception();
	}
	for ( ; i < N; i++) {
		x[i].re = (fftc_real)0;
		x[i].im = (fftc_real)0;
	}
}

// Given a not too large floating point number, round it to the nearest integer.
static inline fftc_real fftc_fround (fftc_real x)
{
	return
	  #if (fftc_real_rounds == rounds_to_nearest)
	    (x + (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2)))
	    - (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2));
	  #elif (fftc_real_rounds == rounds_to_infinity)
	    (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2))
	    - ((fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2))
	       - (x + (fftc_real)0.5));
	  #else // rounds_to_zero, rounds_to_minus_infinity
	    ((x + (fftc_real)0.5)
	     + (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2)))
	    - (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2));
	  #endif
}

// Given a not too large floating point number, round it down.
static inline fftc_real fftc_ffloor (fftc_real x)
{
	#if (fftc_real_rounds == rounds_to_nearest)
	  var fftc_real y =
	    (x + (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2)))
	    - (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2));
	  if (y <= x)
		return y;
	  else
		return y - (fftc_real)1.0;
	#elif (fftc_real_rounds == rounds_to_infinity)
	  return
	    (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2))
	    - ((fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2))
	       - x);
	#else // rounds_to_zero, rounds_to_minus_infinity
	  return
	    (x + (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2)))
	    - (fftc_pow2(fftc_real_mant_bits-1)+fftc_pow2(fftc_real_mant_bits-2));
	#endif
}

// Combine the N complex numbers at z into uintD's below destptr.
// The z[i] are known to be approximately integers >= 0, < N*2^(2*l).
// Assumes room for floor(N*l/intDsize)+(1+ceiling((n+2*l)/intDsize)) uintD's
// below destptr. Fills len digits and returns (destptr lspop len).
static uintD* unfill_product (uintL n, uintC N, // N = 2^n
                              const fftc_complex * z, uintL l,
                              uintD* destptr)
{
	var uintC i;
	if (n + 2*l <= intDsize) {
		// 2-digit carry is sufficient, l < intDsize
		var uintD carry0 = 0;
		var uintD carry1 = 0;
		var uintL shift = 0; // shift next digit before adding it to the carry, >=0, <intDsize
		for (i = 0; i < N; i++) {
			// Fetch z[i].re and round it to the nearest uintD.
			var uintD digit = (uintD)(z[i].re + (fftc_real)0.5);
			if (shift > 0) {
				carry1 += digit >> (intDsize-shift);
				digit = digit << shift;
			}
			if ((carry0 += digit) < digit)
				carry1 += 1;
			shift += l;
			if (shift >= intDsize) {
				lsprefnext(destptr) = carry0;
				carry0 = carry1;
				carry1 = 0;
				shift -= intDsize;
			}
		}
		lsprefnext(destptr) = carry0;
		lsprefnext(destptr) = carry1;
	} else if (n + 2*l <= 2*intDsize) {
		// 3-digit carry is sufficient, l < intDsize
		#if HAVE_DD
		var uintDD carry0 = 0;
		var uintD carry1 = 0;
		var uintL shift = 0; // shift next digit before adding it to the carry, >=0, <intDsize
		for (i = 0; i < N; i++) {
			// Fetch z[i].re and round it to the nearest uintDD.
			var uintDD digit = (uintDD)(z[i].re + (fftc_real)0.5);
			if (shift > 0) {
				carry1 += (uintD)(digit >> (2*intDsize-shift));
				digit = digit << shift;
			}
			if ((carry0 += digit) < digit)
				carry1 += 1;
			shift += l;
			if (shift >= intDsize) {
				lsprefnext(destptr) = lowD(carry0);
				carry0 = highlowDD(carry1,highD(carry0));
				carry1 = 0;
				shift -= intDsize;
			}
		}
		lsprefnext(destptr) = lowD(carry0);
		lsprefnext(destptr) = highD(carry0);
		lsprefnext(destptr) = carry1;
		#else
		var uintD carry0 = 0;
		var uintD carry1 = 0;
		var uintD carry2 = 0;
		var uintL shift = 0; // shift next digit before adding it to the carry, >=0, <intDsize
		for (i = 0; i < N; i++) {
			// Fetch z[i].re and round it to the nearest uintDD.
			var fftc_real zi = z[i].re + (fftc_real)0.5;
			var const fftc_real pow2_intDsize = fftc_pow2(intDsize);
			var const fftc_real invpow2_intDsize = (fftc_real)1.0/pow2_intDsize;
			var uintD digit1 = (uintD)(zi * invpow2_intDsize);
			var uintD digit0 = (uintD)(zi - (fftc_real)digit1 * pow2_intDsize);
			if (shift > 0) {
				carry2 += digit1 >> (intDsize-shift);
				digit1 = (digit1 << shift) | (digit0 >> (intDsize-shift));
				digit0 = digit0 << shift;
			}
			if ((carry0 += digit0) < digit0)
				if ((carry1 += 1) == 0)
					carry2 += 1;
			if ((carry1 += digit1) < digit1)
				carry2 += 1;
			shift += l;
			if (shift >= intDsize) {
				lsprefnext(destptr) = carry0;
				carry0 = carry1;
				carry1 = carry2;
				carry2 = 0;
				shift -= intDsize;
			}
		}
		lsprefnext(destptr) = carry0;
		lsprefnext(destptr) = carry1;
		lsprefnext(destptr) = carry2;
		#endif
	} else {
		// 1-digit+1-float carry is sufficient
		var uintD carry0 = 0;
		var fftc_real carry1 = 0;
		var uintL shift = 0; // shift next digit before adding it to the carry, >=0, <intDsize
		for (i = 0; i < N; i++) {
			// Fetch z[i].re and round it to the nearest integer.
			var fftc_real digit = fftc_fround(z[i].re);
			#ifdef DEBUG_FFTC
			if (!(digit >= (fftc_real)0
			      && z[i].re > digit - (fftc_real)0.5
			      && z[i].re < digit + (fftc_real)0.5))
				throw runtime_exception();
			#endif
			if (shift > 0)
				digit = digit * fftc_pow2_table[shift];
			var fftc_real digit1 = fftc_ffloor(digit*((fftc_real)1.0/fftc_pow2(intDsize)));
			var uintD digit0 = (uintD)(digit - digit1*fftc_pow2(intDsize));
			carry1 += digit1;
			if ((carry0 += digit0) < digit0)
				carry1 += (fftc_real)1.0;
			shift += l;
			while (shift >= intDsize) {
				lsprefnext(destptr) = carry0;
				var fftc_real tmp = fftc_ffloor(carry1*((fftc_real)1.0/fftc_pow2(intDsize)));
				carry0 = (uintD)(carry1 - tmp*fftc_pow2(intDsize));
				carry1 = tmp;
				shift -= intDsize;
			}
		}
		if (carry0 > 0 || carry1 > (fftc_real)0.0) {
			lsprefnext(destptr) = carry0;
			while (carry1 > (fftc_real)0.0) {
				var fftc_real tmp = fftc_ffloor(carry1*((fftc_real)1.0/fftc_pow2(intDsize)));
				lsprefnext(destptr) = (uintD)(carry1 - tmp*fftc_pow2(intDsize));
				carry1 = tmp;
			}
		}
	}
	return destptr;
}

static inline void mulu_fftcomplex_nocheck (const uintD* sourceptr1, uintC len1,
                                            const uintD* sourceptr2, uintC len2,
                                            uintD* destptr)
// Es ist 2 <= len1 <= len2.
{
	// We have to find parameters l and k such that
	// ceiling(len1*intDsize/l) + ceiling(len2*intDsize/l) - 1 <= 2^k,
	// and (12*k-15)*e*2^(2*l+2*k+1/2-m) < 1/2.
	// Try primarily to minimize k. Minimizing l buys you nothing.
	var uintL k;
	// Computing k: If len1 and len2 differ much, we'll split source2 -
	// hence for the moment just substitute len1 for len2.
	//
	// First approximation of k: A necessary condition for
	// 2*ceiling(len1*intDsize/l) - 1 <= 2^k
	// is  2*len1*intDsize/l_max - 1 <= 2^k.
	{
		var const int l = max_l(2);
		var uintC lhs = 2*ceiling(len1*intDsize,l) - 1; // >=1
		if (lhs < 3)
			k = 2;
		else
			integerlengthC(lhs-1, k=); // k>=2
	}
	// Try whether this k is ok or whether we have to increase k.
	for ( ; ; k++) {
		if (k >= sizeof(max_l_table)/sizeof(max_l_table[0])
		    || max_l_table[k] <= 0) {
			throw runtime_exception("FFT problem: numbers too big, floating point precision not sufficient");
		}
		if (2*ceiling(len1*intDsize,max_l_table[k])-1 <= ((uintC)1 << k))
			break;
	}
	// We could try to reduce l, keeping the same k. But why should we?
	// Calculate the number of pieces in which source2 will have to be
	// split. Each of the pieces must satisfy
	// ceiling(len1*intDsize/l) + ceiling(len2*intDsize/l) - 1 <= 2^k,
	var uintC len2p;
	// Try once with k, once with k+1. Compare them.
	{
		var uintC remaining_k = ((uintC)1 << k) + 1 - ceiling(len1*intDsize,max_l_table[k]);
		var uintC max_piecelen_k = floor(remaining_k*max_l_table[k],intDsize);
		var uintC numpieces_k = ceiling(len2,max_piecelen_k);
		var uintC remaining_k1 = ((uintC)1 << (k+1)) + 1 - ceiling(len1*intDsize,max_l_table[k+1]);
		var uintC max_piecelen_k1 = floor(remaining_k1*max_l_table[k+1],intDsize);
		var uintC numpieces_k1 = ceiling(len2,max_piecelen_k1);
		if (numpieces_k <= 2*numpieces_k1) {
			// keep k
			len2p = max_piecelen_k;
		} else {
			// choose k+1
			k = k+1;
			len2p = max_piecelen_k1;
		}
	}
	var const uintL l = max_l_table[k];
	var const uintL n = k;
	var const uintC N = (uintC)1 << n;
	CL_ALLOCA_STACK;
	var fftc_complex* const x = cl_alloc_array(fftc_complex,N);
	var fftc_complex* const y = cl_alloc_array(fftc_complex,N);
	#ifdef DEBUG_FFTC
	var fftc_complex* const z = cl_alloc_array(fftc_complex,N);
	#else
	var fftc_complex* const z = x; // put z in place of x - saves memory
	#endif
	var uintD* const tmpprod1 = cl_alloc_array(uintD,len1+1);
	var uintC tmpprod_len = floor(l<<n,intDsize)+6;
	var uintD* const tmpprod = cl_alloc_array(uintD,tmpprod_len);
	var uintC destlen = len1+len2;
	clear_loop_lsp(destptr,destlen);
	do {
		if (len2p > len2)
			len2p = len2;
		if (len2p == 1) {
			// cheap case
			var uintD* tmpptr = arrayLSDptr(tmpprod1,len1+1);
			mulu_loop_lsp(lspref(sourceptr2,0),sourceptr1,tmpptr,len1);
			if (addto_loop_lsp(tmpptr,destptr,len1+1))
				if (inc_loop_lsp(destptr lspop (len1+1),destlen-(len1+1)))
					throw runtime_exception();
		} else {
			var bool squaring = ((sourceptr1 == sourceptr2) && (len1 == len2p));
			// Fill factor x.
			fill_factor(N,x,l,sourceptr1,len1);
			// Fill factor y.
			if (!squaring)
				fill_factor(N,y,l,sourceptr2,len2p);
			// Multiply.
			if (!squaring)
				fftc_convolution(n,N, &x[0], &y[0], &z[0]);
			else
				fftc_convolution(n,N, &x[0], &x[0], &z[0]);
			#ifdef DEBUG_FFTC
			// Check result.
			{
				var fftc_real re_lo_limit = (fftc_real)(-0.5);
				var fftc_real re_hi_limit = (fftc_real)N * fftc_pow2_table[l] * fftc_pow2_table[l] + (fftc_real)0.5;
				var fftc_real im_lo_limit = (fftc_real)(-0.5);
				var fftc_real im_hi_limit = (fftc_real)0.5;
				for (var uintC i = 0; i < N; i++) {
					if (!(z[i].im > im_lo_limit
					      && z[i].im < im_hi_limit))
						throw runtime_exception();
					if (!(z[i].re > re_lo_limit
					      && z[i].re < re_hi_limit))
						throw runtime_exception();
				}
			}
			#endif
			var uintD* tmpLSDptr = arrayLSDptr(tmpprod,tmpprod_len);
			var uintD* tmpMSDptr = unfill_product(n,N,z,l,tmpLSDptr);
			var uintC tmplen =
			  #if CL_DS_BIG_ENDIAN_P
			    tmpLSDptr - tmpMSDptr;
			  #else
			    tmpMSDptr - tmpLSDptr;
			  #endif
			if (tmplen > tmpprod_len)
			  throw runtime_exception();
			// Add result to destptr[-destlen..-1]:
			if (tmplen > destlen) {
				if (test_loop_msp(tmpMSDptr,tmplen-destlen))
					throw runtime_exception();
				tmplen = destlen;
			}
			if (addto_loop_lsp(tmpLSDptr,destptr,tmplen))
				if (inc_loop_lsp(destptr lspop tmplen,destlen-tmplen))
					throw runtime_exception();
		}
		// Decrement len2.
		destptr = destptr lspop len2p;
		destlen -= len2p;
		sourceptr2 = sourceptr2 lspop len2p;
		len2 -= len2p;
	} while (len2 > 0);
}

#ifndef _CHECKSUM
#define _CHECKSUM

// Compute a checksum: number mod (2^intDsize-1).
static uintD compute_checksum (const uintD* sourceptr, uintC len)
{
	var uintD tmp = ~(uintD)0; // -1-(sum mod 2^intDsize-1), always >0
	do {
		var uintD digit = lsprefnext(sourceptr);
		if (digit < tmp)
			tmp -= digit; // subtract digit
		else
			tmp -= digit+1; // subtract digit-(2^intDsize-1)
	} while (--len > 0);
	return ~tmp;
}

// Multiply two checksums modulo (2^intDsize-1).
static inline uintD multiply_checksum (uintD checksum1, uintD checksum2)
{
	var uintD checksum;
	var uintD cksum_hi;
	#if HAVE_DD
	var uintDD cksum = muluD(checksum1,checksum2);
	cksum_hi = highD(cksum); checksum = lowD(cksum);
	#else
	muluD(checksum1,checksum2, cksum_hi =, checksum =);
	#endif
	if ((checksum += cksum_hi) + 1 <= cksum_hi)
		checksum += 1;
	return checksum;
}

#endif // _CHECKSUM

static void mulu_fftcomplex (const uintD* sourceptr1, uintC len1,
                             const uintD* sourceptr2, uintC len2,
                             uintD* destptr)
{
	// Compute checksums of the arguments and multiply them.
	var uintD checksum1 = compute_checksum(sourceptr1,len1);
	var uintD checksum2 = compute_checksum(sourceptr2,len2);
	var uintD checksum = multiply_checksum(checksum1,checksum2);
	mulu_fftcomplex_nocheck(sourceptr1,len1,sourceptr2,len2,destptr);
	if (!(checksum == compute_checksum(destptr,len1+len2))) {
		throw runtime_exception("FFT problem: checksum error");
	}
}
