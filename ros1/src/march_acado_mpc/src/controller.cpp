// !Unfinished!


#include "../include/march_acado_mpc/mpc.h"

#include <iostream>

using namespace std;

const bool VERBOSE = 1; // Print debug information: 1, silent: 0.

void ModelPredictiveController::initSolver( )
{
    // Reset all solver memory
    memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    memset(&acadoVariables, 0, sizeof( acadoVariables ));

    //
    // Initialize the solver
    //
    acado_initializeSolver();

    //
    // Prepare a consistent initial guess
    //
    for (int i = 0; i < N + 1; ++i)
    {
        acadoVariables.x[i * NX + 0] = 0;
        acadoVariables.x[i * NX + 1] = 0;
    }

    //
    // Prepare references
    //
    for (int i = 0; i < N; ++i)
    {
        acadoVariables.y[i * NY + 0] = 0; // x1
        acadoVariables.y[i * NY + 1] = 0; // x2
        acadoVariables.y[i * NY + 2] = 0; // u
    }

    acadoVariables.yN[ 0 ] = 0; // x1
    acadoVariables.yN[ 1 ] = 0; // x2
    acadoVariables.yN[ 2 ] = 0; // u


}

void ModelPredictiveController::setInitState(double x0[NX])
{
    for (int i = 0; i < NX; ++i)
    {
        acadoVariables.x0[i] = x0[i];
    }
}

void ModelPredictiveController::printDebug(const bool VERBOSE)
{
    if (VERBOSE)
    {
        cout << "\nDebug Information\n-----------------\n";
        cout << "x0 = [" << acadoVariables.x0[0] << ", " << acadoVariables.x0[1] << "]" << endl;
        acado_printDifferentialVariables();
        acado_printControlVariables();
    }
}

void ModelPredictiveController::controller(double x0[NX])
{
    // return the States and Control sequences
    // return the iteration time

    // setup iteration timer
    real_t t_iter;

    acado_timer t;
    acado_tic( &t );

    //
    // Warm-up the solver
    //
    acado_preparationStep();

    // set initial state from measurements
    ModelPredictiveController::setInitState(x0);

    acado_feedbackStep( );
    t_iter = acado_toc( &t );

    cout << t_iter * 1e6 << " microseconds" << endl;

}

int main( )
{
    // declare some variables

    ModelPredictiveController mpc;
    mpc.initSolver();

    double x0[NX] = {1.0,2.0};
    mpc.controller(x0);

    mpc.printDebug(VERBOSE);

    return 0;
}


