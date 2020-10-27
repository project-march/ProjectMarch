/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 9 */
real_t state[ 9 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 2];
state[1] = acadoVariables.x[lRun1 * 2 + 1];

state[8] = acadoVariables.u[lRun1];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 2] = state[0] - acadoVariables.x[lRun1 * 2 + 2];
acadoWorkspace.d[lRun1 * 2 + 1] = state[1] - acadoVariables.x[lRun1 * 2 + 3];

acadoWorkspace.evGx[lRun1 * 4] = state[2];
acadoWorkspace.evGx[lRun1 * 4 + 1] = state[3];
acadoWorkspace.evGx[lRun1 * 4 + 2] = state[4];
acadoWorkspace.evGx[lRun1 * 4 + 3] = state[5];

acadoWorkspace.evGu[lRun1 * 2] = state[6];
acadoWorkspace.evGu[lRun1 * 2 + 1] = state[7];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 2;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[3];
tmpQ1[3] = + tmpQ2[4];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[6];
tmpR2[1] = +tmpObjS[7];
tmpR2[2] = +tmpObjS[8];
tmpR1[0] = + tmpR2[2];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 3] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 3 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 3 + 2] = acadoWorkspace.objValueOut[2];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 4 ]), &(acadoWorkspace.Q2[ runObj * 6 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj ]), &(acadoWorkspace.R2[ runObj * 3 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[20];
acadoWorkspace.objValueIn[1] = acadoVariables.x[21];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1];
Gu2[1] = + Gx1[2]*Gu1[0] + Gx1[3]*Gu1[1];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 11] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + R11[0];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[2]*Gu1[1];
Gu2[1] = + Gx1[1]*Gu1[0] + Gx1[3]*Gu1[1];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[1] + Gu2[0];
Gu3[1] = + Q11[2]*Gu1[0] + Q11[3]*Gu1[1] + Gu2[1];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[2]*w11[1] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[3]*w11[1] + w12[1];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[1]*w11[1];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + w12[0];
w13[1] = + Q11[2]*w11[0] + Q11[3]*w11[1] + w12[1];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1];
w12[1] += + Gx1[2]*w11[0] + Gx1[3]*w11[1];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1];
w12[1] += + Gx1[2]*w11[0] + Gx1[3]*w11[1];
w12[0] += + Gu1[0]*U1[0];
w12[1] += + Gu1[1]*U1[0];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = acadoWorkspace.H[(iCol * 10) + (iRow)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2];
QDy1[1] = + Q2[3]*Dy1[0] + Q2[4]*Dy1[1] + Q2[5]*Dy1[2];
}

void acado_condensePrep(  )
{
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 2 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.E[ 2 ]), &(acadoWorkspace.E[ 4 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.E[ 6 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.E[ 8 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.E[ 10 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.E[ 12 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 14 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 14 ]), &(acadoWorkspace.E[ 16 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.E[ 18 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 18 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 16 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 8, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 14 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.W1, 7, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.E[ 12 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.W1, 6, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 24 ]), &(acadoWorkspace.E[ 10 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.W1, 5, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 20 ]), &(acadoWorkspace.E[ 8 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.E[ 6 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.W1, 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.E[ 4 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.W1, 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 8 ]), &(acadoWorkspace.E[ 2 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 2 ]), acadoWorkspace.W1, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 4 ]), acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.R1, acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 2 ]), &(acadoWorkspace.E[ 20 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.E[ 22 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.E[ 22 ]), &(acadoWorkspace.E[ 24 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 26 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.E[ 26 ]), &(acadoWorkspace.E[ 28 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.E[ 30 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 32 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.E[ 34 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.E[ 34 ]), &(acadoWorkspace.E[ 36 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 36 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 34 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 8, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 32 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.W1, 7, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.E[ 30 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.W1, 6, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 24 ]), &(acadoWorkspace.E[ 28 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.W1, 5, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 20 ]), &(acadoWorkspace.E[ 26 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.E[ 24 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.W1, 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.E[ 22 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.W1, 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 8 ]), &(acadoWorkspace.E[ 20 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 1 ]), &(acadoWorkspace.evGu[ 2 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.E[ 38 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.E[ 38 ]), &(acadoWorkspace.E[ 40 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 42 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.E[ 44 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.E[ 46 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 50 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.E[ 52 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 52 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 50 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 8, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 48 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.W1, 7, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.E[ 46 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.W1, 6, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 24 ]), &(acadoWorkspace.E[ 44 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.W1, 5, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 20 ]), &(acadoWorkspace.E[ 42 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.E[ 40 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.W1, 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.E[ 38 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 2 ]), &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.E[ 54 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.E[ 56 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.E[ 58 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.E[ 60 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 62 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.E[ 64 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.E[ 66 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 66 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 64 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 8, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 62 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.W1, 7, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.E[ 60 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.W1, 6, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 24 ]), &(acadoWorkspace.E[ 58 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.W1, 5, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 20 ]), &(acadoWorkspace.E[ 56 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.E[ 54 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 3 ]), &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.E[ 68 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.E[ 70 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 74 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.E[ 76 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.E[ 78 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 78 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 76 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 8, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 74 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.W1, 7, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.E[ 72 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.W1, 6, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 24 ]), &(acadoWorkspace.E[ 70 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.W1, 5, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 20 ]), &(acadoWorkspace.E[ 68 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 4 ]), &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4 );

/* Column: 5 */
acado_moveGuE( &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.E[ 80 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 82 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.E[ 82 ]), &(acadoWorkspace.E[ 84 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 86 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.E[ 86 ]), &(acadoWorkspace.E[ 88 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 88 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 86 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 8, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 84 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.W1, 7, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.E[ 82 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.W1, 6, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 24 ]), &(acadoWorkspace.E[ 80 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 5 ]), &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.W1, 5 );

/* Column: 6 */
acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 90 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.E[ 92 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.E[ 94 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.E[ 96 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 96 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 94 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 8, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 92 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.W1, 7, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.E[ 90 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 6 ]), &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.W1, 6 );

/* Column: 7 */
acado_moveGuE( &(acadoWorkspace.evGu[ 14 ]), &(acadoWorkspace.E[ 98 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.E[ 100 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.E[ 102 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 102 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 100 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 8, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 98 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 7 ]), &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.W1, 7 );

/* Column: 8 */
acado_moveGuE( &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.E[ 104 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.E[ 106 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 106 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 104 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 8 ]), &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 8 );

/* Column: 9 */
acado_moveGuE( &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.E[ 108 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 108 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 9 ]), &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.W1, 9 );

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );

acadoWorkspace.sbar[2] = acadoWorkspace.d[0];
acadoWorkspace.sbar[3] = acadoWorkspace.d[1];
acadoWorkspace.sbar[4] = acadoWorkspace.d[2];
acadoWorkspace.sbar[5] = acadoWorkspace.d[3];
acadoWorkspace.sbar[6] = acadoWorkspace.d[4];
acadoWorkspace.sbar[7] = acadoWorkspace.d[5];
acadoWorkspace.sbar[8] = acadoWorkspace.d[6];
acadoWorkspace.sbar[9] = acadoWorkspace.d[7];
acadoWorkspace.sbar[10] = acadoWorkspace.d[8];
acadoWorkspace.sbar[11] = acadoWorkspace.d[9];
acadoWorkspace.sbar[12] = acadoWorkspace.d[10];
acadoWorkspace.sbar[13] = acadoWorkspace.d[11];
acadoWorkspace.sbar[14] = acadoWorkspace.d[12];
acadoWorkspace.sbar[15] = acadoWorkspace.d[13];
acadoWorkspace.sbar[16] = acadoWorkspace.d[14];
acadoWorkspace.sbar[17] = acadoWorkspace.d[15];
acadoWorkspace.sbar[18] = acadoWorkspace.d[16];
acadoWorkspace.sbar[19] = acadoWorkspace.d[17];
acadoWorkspace.sbar[20] = acadoWorkspace.d[18];
acadoWorkspace.sbar[21] = acadoWorkspace.d[19];

acadoWorkspace.A[0] = acadoWorkspace.E[0];

acadoWorkspace.A[10] = acadoWorkspace.E[2];
acadoWorkspace.A[11] = acadoWorkspace.E[20];

acadoWorkspace.A[20] = acadoWorkspace.E[4];
acadoWorkspace.A[21] = acadoWorkspace.E[22];
acadoWorkspace.A[22] = acadoWorkspace.E[38];

acadoWorkspace.A[30] = acadoWorkspace.E[6];
acadoWorkspace.A[31] = acadoWorkspace.E[24];
acadoWorkspace.A[32] = acadoWorkspace.E[40];
acadoWorkspace.A[33] = acadoWorkspace.E[54];

acadoWorkspace.A[40] = acadoWorkspace.E[8];
acadoWorkspace.A[41] = acadoWorkspace.E[26];
acadoWorkspace.A[42] = acadoWorkspace.E[42];
acadoWorkspace.A[43] = acadoWorkspace.E[56];
acadoWorkspace.A[44] = acadoWorkspace.E[68];

acadoWorkspace.A[50] = acadoWorkspace.E[10];
acadoWorkspace.A[51] = acadoWorkspace.E[28];
acadoWorkspace.A[52] = acadoWorkspace.E[44];
acadoWorkspace.A[53] = acadoWorkspace.E[58];
acadoWorkspace.A[54] = acadoWorkspace.E[70];
acadoWorkspace.A[55] = acadoWorkspace.E[80];

acadoWorkspace.A[60] = acadoWorkspace.E[12];
acadoWorkspace.A[61] = acadoWorkspace.E[30];
acadoWorkspace.A[62] = acadoWorkspace.E[46];
acadoWorkspace.A[63] = acadoWorkspace.E[60];
acadoWorkspace.A[64] = acadoWorkspace.E[72];
acadoWorkspace.A[65] = acadoWorkspace.E[82];
acadoWorkspace.A[66] = acadoWorkspace.E[90];

acadoWorkspace.A[70] = acadoWorkspace.E[14];
acadoWorkspace.A[71] = acadoWorkspace.E[32];
acadoWorkspace.A[72] = acadoWorkspace.E[48];
acadoWorkspace.A[73] = acadoWorkspace.E[62];
acadoWorkspace.A[74] = acadoWorkspace.E[74];
acadoWorkspace.A[75] = acadoWorkspace.E[84];
acadoWorkspace.A[76] = acadoWorkspace.E[92];
acadoWorkspace.A[77] = acadoWorkspace.E[98];

acadoWorkspace.A[80] = acadoWorkspace.E[16];
acadoWorkspace.A[81] = acadoWorkspace.E[34];
acadoWorkspace.A[82] = acadoWorkspace.E[50];
acadoWorkspace.A[83] = acadoWorkspace.E[64];
acadoWorkspace.A[84] = acadoWorkspace.E[76];
acadoWorkspace.A[85] = acadoWorkspace.E[86];
acadoWorkspace.A[86] = acadoWorkspace.E[94];
acadoWorkspace.A[87] = acadoWorkspace.E[100];
acadoWorkspace.A[88] = acadoWorkspace.E[104];

acadoWorkspace.A[90] = acadoWorkspace.E[18];
acadoWorkspace.A[91] = acadoWorkspace.E[36];
acadoWorkspace.A[92] = acadoWorkspace.E[52];
acadoWorkspace.A[93] = acadoWorkspace.E[66];
acadoWorkspace.A[94] = acadoWorkspace.E[78];
acadoWorkspace.A[95] = acadoWorkspace.E[88];
acadoWorkspace.A[96] = acadoWorkspace.E[96];
acadoWorkspace.A[97] = acadoWorkspace.E[102];
acadoWorkspace.A[98] = acadoWorkspace.E[106];
acadoWorkspace.A[99] = acadoWorkspace.E[108];


}

void acado_condenseFdb(  )
{
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 3 ]), &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 6 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 9 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 12 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 15 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 18 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 21 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 27 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 9 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 6 ]), &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.QDy[ 2 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 12 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 18 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 24 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 30 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 36 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 42 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 14 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 48 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 54 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 18 ]) );

acadoWorkspace.QDy[20] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[21] = + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[1];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 2 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 14 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 20 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[20] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[21] + acadoWorkspace.QDy[20];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[20] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[21] + acadoWorkspace.QDy[21];
acado_macBTw1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 7 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 14 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.sbar[ 14 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 24 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 5 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 20 ]), &(acadoWorkspace.sbar[ 10 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 8 ]), &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 2 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 1 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 2 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4 ]), &(acadoWorkspace.sbar[ 2 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];

tmp = acadoWorkspace.sbar[2] + acadoVariables.x[2];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = acadoWorkspace.sbar[4] + acadoVariables.x[4];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = acadoWorkspace.sbar[6] + acadoVariables.x[6];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = acadoWorkspace.sbar[8] + acadoVariables.x[8];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = acadoWorkspace.sbar[10] + acadoVariables.x[10];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = acadoWorkspace.sbar[12] + acadoVariables.x[12];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;
tmp = acadoWorkspace.sbar[14] + acadoVariables.x[14];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - tmp;
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - tmp;
tmp = acadoWorkspace.sbar[16] + acadoVariables.x[16];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - tmp;
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - tmp;
tmp = acadoWorkspace.sbar[18] + acadoVariables.x[18];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - tmp;
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - tmp;
tmp = acadoWorkspace.sbar[20] + acadoVariables.x[20];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - tmp;
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - tmp;

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.d[0];
acadoWorkspace.sbar[3] = acadoWorkspace.d[1];
acadoWorkspace.sbar[4] = acadoWorkspace.d[2];
acadoWorkspace.sbar[5] = acadoWorkspace.d[3];
acadoWorkspace.sbar[6] = acadoWorkspace.d[4];
acadoWorkspace.sbar[7] = acadoWorkspace.d[5];
acadoWorkspace.sbar[8] = acadoWorkspace.d[6];
acadoWorkspace.sbar[9] = acadoWorkspace.d[7];
acadoWorkspace.sbar[10] = acadoWorkspace.d[8];
acadoWorkspace.sbar[11] = acadoWorkspace.d[9];
acadoWorkspace.sbar[12] = acadoWorkspace.d[10];
acadoWorkspace.sbar[13] = acadoWorkspace.d[11];
acadoWorkspace.sbar[14] = acadoWorkspace.d[12];
acadoWorkspace.sbar[15] = acadoWorkspace.d[13];
acadoWorkspace.sbar[16] = acadoWorkspace.d[14];
acadoWorkspace.sbar[17] = acadoWorkspace.d[15];
acadoWorkspace.sbar[18] = acadoWorkspace.d[16];
acadoWorkspace.sbar[19] = acadoWorkspace.d[17];
acadoWorkspace.sbar[20] = acadoWorkspace.d[18];
acadoWorkspace.sbar[21] = acadoWorkspace.d[19];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 2 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.evGu[ 2 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.x[ 5 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 14 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.evGu[ 14 ]), &(acadoWorkspace.x[ 7 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 20 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -5.0000000000000000e+01;
acadoVariables.lbValues[1] = -5.0000000000000000e+01;
acadoVariables.lbValues[2] = -5.0000000000000000e+01;
acadoVariables.lbValues[3] = -5.0000000000000000e+01;
acadoVariables.lbValues[4] = -5.0000000000000000e+01;
acadoVariables.lbValues[5] = -5.0000000000000000e+01;
acadoVariables.lbValues[6] = -5.0000000000000000e+01;
acadoVariables.lbValues[7] = -5.0000000000000000e+01;
acadoVariables.lbValues[8] = -5.0000000000000000e+01;
acadoVariables.lbValues[9] = -5.0000000000000000e+01;
acadoVariables.ubValues[0] = 5.0000000000000000e+01;
acadoVariables.ubValues[1] = 5.0000000000000000e+01;
acadoVariables.ubValues[2] = 5.0000000000000000e+01;
acadoVariables.ubValues[3] = 5.0000000000000000e+01;
acadoVariables.ubValues[4] = 5.0000000000000000e+01;
acadoVariables.ubValues[5] = 5.0000000000000000e+01;
acadoVariables.ubValues[6] = 5.0000000000000000e+01;
acadoVariables.ubValues[7] = 5.0000000000000000e+01;
acadoVariables.ubValues[8] = 5.0000000000000000e+01;
acadoVariables.ubValues[9] = 5.0000000000000000e+01;
acadoVariables.lbAValues[0] = -1.5707963267948966e+00;
acadoVariables.lbAValues[1] = -1.5707963267948966e+00;
acadoVariables.lbAValues[2] = -1.5707963267948966e+00;
acadoVariables.lbAValues[3] = -1.5707963267948966e+00;
acadoVariables.lbAValues[4] = -1.5707963267948966e+00;
acadoVariables.lbAValues[5] = -1.5707963267948966e+00;
acadoVariables.lbAValues[6] = -1.5707963267948966e+00;
acadoVariables.lbAValues[7] = -1.5707963267948966e+00;
acadoVariables.lbAValues[8] = -1.5707963267948966e+00;
acadoVariables.lbAValues[9] = -1.5707963267948966e+00;
acadoVariables.ubAValues[0] = 1.5707963267948966e+00;
acadoVariables.ubAValues[1] = 1.5707963267948966e+00;
acadoVariables.ubAValues[2] = 1.5707963267948966e+00;
acadoVariables.ubAValues[3] = 1.5707963267948966e+00;
acadoVariables.ubAValues[4] = 1.5707963267948966e+00;
acadoVariables.ubAValues[5] = 1.5707963267948966e+00;
acadoVariables.ubAValues[6] = 1.5707963267948966e+00;
acadoVariables.ubAValues[7] = 1.5707963267948966e+00;
acadoVariables.ubAValues[8] = 1.5707963267948966e+00;
acadoVariables.ubAValues[9] = 1.5707963267948966e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
state[0] = acadoVariables.x[index * 2];
state[1] = acadoVariables.x[index * 2 + 1];
state[8] = acadoVariables.u[index];

acado_integrate(state, index == 0);

acadoVariables.x[index * 2 + 2] = state[0];
acadoVariables.x[index * 2 + 3] = state[1];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 2] = acadoVariables.x[index * 2 + 2];
acadoVariables.x[index * 2 + 1] = acadoVariables.x[index * 2 + 3];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[20] = xEnd[0];
acadoVariables.x[21] = xEnd[1];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[20];
state[1] = acadoVariables.x[21];
if (uEnd != 0)
{
state[8] = uEnd[0];
}
else
{
state[8] = acadoVariables.u[9];
}

acado_integrate(state, 1);

acadoVariables.x[20] = state[0];
acadoVariables.x[21] = state[1];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[9] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9];
kkt = fabs( kkt );
for (index = 0; index < 10; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 10; ++index)
{
prd = acadoWorkspace.y[index + 10];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 3 */
real_t tmpDy[ 3 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 3] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 3];
acadoWorkspace.Dy[lRun1 * 3 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 3 + 1];
acadoWorkspace.Dy[lRun1 * 3 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 3 + 2];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[20];
acadoWorkspace.objValueIn[1] = acadoVariables.x[21];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[4];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[8];
objVal += + acadoWorkspace.Dy[lRun1 * 3]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 3 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 3 + 2]*tmpDy[2];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[3];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

