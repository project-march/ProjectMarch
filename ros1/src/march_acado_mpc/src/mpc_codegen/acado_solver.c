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
for (lRun1 = 0; lRun1 < 5; ++lRun1)
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

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 5; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 3] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 3 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 3 + 2] = acadoWorkspace.objValueOut[2];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[11];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

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
acadoWorkspace.H[(iRow * 5) + (iCol)] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1];
}

void acado_multBTW1_R1( real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 6] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + (real_t)1.0000000000000000e-02;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[2]*Gu1[1];
Gu2[1] = + Gx1[1]*Gu1[0] + Gx1[3]*Gu1[1];
}

void acado_multQEW2( real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + (real_t)1.0000000000000000e+02*Gu1[0] + Gu2[0];
Gu3[1] = +Gu1[1] + Gu2[1];
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

void acado_macQSbarW2( real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + (real_t)1.0000000000000000e+02*w11[0] + w12[0];
w13[1] = +w11[1] + w12[1];
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
acadoWorkspace.H[(iRow * 5) + (iCol)] = acadoWorkspace.H[(iCol * 5) + (iRow)];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + (real_t)1.0000000000000000e-02*Dy1[2];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)1.0000000000000000e+02*Dy1[0];
QDy1[1] = +Dy1[1];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e+02*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e+02*Gx1[1];
Gx2[2] = +Gx1[2];
Gx2[3] = +Gx1[3];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e+02*Gu1[0];
Gu2[1] = +Gu1[1];
}

void acado_condensePrep(  )
{
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 2 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.E[ 2 ]), &(acadoWorkspace.E[ 4 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.E[ 6 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.E[ 8 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 8 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 6 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.W1, 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.W1, 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 2 ]), acadoWorkspace.W1, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 2 ]), &(acadoWorkspace.E[ 10 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.E[ 12 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 14 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 14 ]), &(acadoWorkspace.E[ 16 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 16 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 14 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.W1, 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 12 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.W1, 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 10 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 2 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.E[ 18 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.E[ 20 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.E[ 22 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 22 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 20 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.W1, 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 18 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.E[ 24 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 26 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 26 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 24 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.E[ 28 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 28 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 4 );

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

acadoWorkspace.A[0] = acadoWorkspace.E[0];

acadoWorkspace.A[5] = acadoWorkspace.E[2];
acadoWorkspace.A[6] = acadoWorkspace.E[10];

acadoWorkspace.A[10] = acadoWorkspace.E[4];
acadoWorkspace.A[11] = acadoWorkspace.E[12];
acadoWorkspace.A[12] = acadoWorkspace.E[18];

acadoWorkspace.A[15] = acadoWorkspace.E[6];
acadoWorkspace.A[16] = acadoWorkspace.E[14];
acadoWorkspace.A[17] = acadoWorkspace.E[20];
acadoWorkspace.A[18] = acadoWorkspace.E[24];

acadoWorkspace.A[20] = acadoWorkspace.E[8];
acadoWorkspace.A[21] = acadoWorkspace.E[16];
acadoWorkspace.A[22] = acadoWorkspace.E[22];
acadoWorkspace.A[23] = acadoWorkspace.E[26];
acadoWorkspace.A[24] = acadoWorkspace.E[28];


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
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.QDy[ 2 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 8 ]) );

acadoWorkspace.QDy[10] = + (real_t)1.0000000000000000e+02*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[11] = +acadoWorkspace.DyN[1];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 2 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 10 ]) );

acadoWorkspace.w1[0] = + (real_t)1.0000000000000000e+02*acadoWorkspace.sbar[10] + acadoWorkspace.QDy[10];
acadoWorkspace.w1[1] = +acadoWorkspace.sbar[11] + acadoWorkspace.QDy[11];
acado_macBTw1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 2 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 1 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 2 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 2 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];

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

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
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
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 2 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.evGu[ 2 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 10 ]) );
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
acadoVariables.ubValues[0] = 5.0000000000000000e+01;
acadoVariables.ubValues[1] = 5.0000000000000000e+01;
acadoVariables.ubValues[2] = 5.0000000000000000e+01;
acadoVariables.ubValues[3] = 5.0000000000000000e+01;
acadoVariables.ubValues[4] = 5.0000000000000000e+01;
acadoVariables.lbAValues[0] = -1.5707963267948966e+00;
acadoVariables.lbAValues[1] = -1.5707963267948966e+00;
acadoVariables.lbAValues[2] = -1.5707963267948966e+00;
acadoVariables.lbAValues[3] = -1.5707963267948966e+00;
acadoVariables.lbAValues[4] = -1.5707963267948966e+00;
acadoVariables.ubAValues[0] = 1.5707963267948966e+00;
acadoVariables.ubAValues[1] = 1.5707963267948966e+00;
acadoVariables.ubAValues[2] = 1.5707963267948966e+00;
acadoVariables.ubAValues[3] = 1.5707963267948966e+00;
acadoVariables.ubAValues[4] = 1.5707963267948966e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 5; ++index)
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
for (index = 0; index < 5; ++index)
{
acadoVariables.x[index * 2] = acadoVariables.x[index * 2 + 2];
acadoVariables.x[index * 2 + 1] = acadoVariables.x[index * 2 + 3];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[10] = xEnd[0];
acadoVariables.x[11] = xEnd[1];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[10];
state[1] = acadoVariables.x[11];
if (uEnd != 0)
{
state[8] = uEnd[0];
}
else
{
state[8] = acadoVariables.u[4];
}

acado_integrate(state, 1);

acadoVariables.x[10] = state[0];
acadoVariables.x[11] = state[1];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 4; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[4] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4];
kkt = fabs( kkt );
for (index = 0; index < 5; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 5; ++index)
{
prd = acadoWorkspace.y[index + 5];
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

for (lRun1 = 0; lRun1 < 5; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 3] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 3];
acadoWorkspace.Dy[lRun1 * 3 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 3 + 1];
acadoWorkspace.Dy[lRun1 * 3 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 3 + 2];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[11];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 5; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 3]*(real_t)1.0000000000000000e+02;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 3 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 3 + 2]*(real_t)1.0000000000000000e-02;
objVal += + acadoWorkspace.Dy[lRun1 * 3]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 3 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 3 + 2]*tmpDy[2];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000000e+02;
tmpDyN[1] = + acadoWorkspace.DyN[1];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

