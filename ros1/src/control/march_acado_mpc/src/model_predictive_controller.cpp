#define _USE_MATH_DEFINES

#include "model_predictive_controller.hpp"
#include "acado_common.h"
#include "mpc_references.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

// Global variables used by the solver
ACADOvariables acadoVariables = {};
ACADOworkspace acadoWorkspace = {};

ModelPredictiveController::ModelPredictiveController(std::vector<std::vector<float>> Q) : Q_(Q)
{
}

void ModelPredictiveController::init()
{
  // Initialize the solver
  acado_initializeSolver();

  // Prepare a consistent initial guess
  for (int i = 0; i <= ACADO_N + 1; i++)
  {
    acadoVariables.x[i * ACADO_NX + 0] = 0;  // theta
    acadoVariables.x[i * ACADO_NX + 1] = 0;  // dtheta
  }

  // Fill reference vector with sinus and or step signals
//  sinRef(reference, 0.1, 0.261791667, ACADO_N, 0.02);  // freq, amp, horizon, sampling time   [10s]
//  sinRef(reference, 0.125, 0.261791667, ACADO_N, 0.02);  // freq, amp, horizon, sampling time [08s]
//  sinRef(reference, 0.16, 0.261791667, ACADO_N, 0.02);  // freq, amp, horizon, sampling time  [06s]
    stepRef(reference, 0.0, 2*ACADO_N);

  // hold last N reference values
//  repeat_reference = false;

  // Set the reference
  setReference(reference);

  //  // Set angle step reference value
  //  double theta_ref = 30*(M_PI/180);
  //
  //  // Prepare references (step reference)
  //  for (int i = 0; i < ACADO_N; ++i) {
  //    acadoVariables.y[i * ACADO_NY] = theta_ref; // theta
  //    acadoVariables.y[i * ACADO_NY + 1] = 0;         // dtheta
  //    acadoVariables.y[i * ACADO_NY + 2] = 0;         // T
  //  }
  //
  //  acadoVariables.yN[0] = theta_ref; // theta
  //  acadoVariables.yN[1] = 0;         // dtheta
  //  acadoVariables.yN[2] = 0;         // T

  // Current state feedback
  setInitialState(x0);

  assignWeightingMatrix(Q_);

  // Warm-up the solver
  acado_preparationStep();
}

void ModelPredictiveController::setInitialState(vector<double> x0)
{
  for (int i = 0; i < ACADO_NX; ++i)
  {
    acadoVariables.x0[i] = x0[i];
  }
}

void ModelPredictiveController::setReference(vector<vector<double>> reference)
{
  for (int i = 0; i < ACADO_N; i++)
  {
    for (int j = 0; j < ACADO_NY; j++)
    {
      acadoVariables.y[i * ACADO_NY + j] = reference[i][j];
    }
  }
  for (int j = 0; j < ACADO_NYN; j++)
  {
    acadoVariables.yN[j] = reference[ACADO_N][j];
  }
}

void ModelPredictiveController::assignWeightingMatrix(std::vector<std::vector<float>> Q)
{
  double ACADO_NW = sizeof(acadoVariables.W) / sizeof(acadoVariables.W[0]);
  double ACADO_NWN = sizeof(acadoVariables.WN) / sizeof(acadoVariables.WN[0]);

  std::cout << ACADO_NW << ", " << ACADO_NWN << std::endl;

  int nrows = Q.size();
  int ncols = Q[0].size();

  // set W matrix with Q matrix
  for (int i = 0; i < nrows; i++)
  {
    for (int j = 0; j < ncols; j++)
    {
      acadoVariables.W[i * ncols + j] = Q[i][j];
    }
  }

  // Set WN matrix with Q matrix
  for (int i = 0; i < (nrows - 1); i++)
  {
    for (int j = 0; j < 2; j++)
    {
      acadoVariables.WN[i * (ncols - 1) + j] = Q[i][j];
    }
  }

  for (int i = 0; i < ACADO_NW; i++)
  {
    std::cout << acadoVariables.W[i] << std::endl;
  }

  for (int i = 0; i < ACADO_NWN; i++)
  {
    std::cout << acadoVariables.WN[i] << std::endl;
  }
}

void ModelPredictiveController::calculateControlInput()
{
  // Set initial state
  setInitialState(x0);
  pos_ref = acadoVariables.y[0];

  // preparation step
  setReference(reference);
  acado_preparationStep();

  // feedback step
  if (acado_feedbackStep())
  {
    std::cout << "\033[4;32m"
              << "Feedback step failed"
              << "()\033[0m" << std::endl;
  }
  u = acadoVariables.u[0];

  // only for discrete time model with integrator
  //  u = u + acadoVariables.u[0];

  std::cout << "MPC COMMAND: " << u << std::endl;
  std::cout << "MPC Cost: " << acado_getObjective() << std::endl;

  // Shift states and control and prepare for the next iteration
  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  // Scroll the reference vector
  if (repeat_reference)
  {
    scrollReference(reference);
  }
}
