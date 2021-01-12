#include "model_predictive_controller.hpp"
#include <vector>

#include <iostream>
using namespace std;

void ModelPredictiveController::initSolver() {
  // Reset all solver memory
  memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  memset(&acadoVariables, 0, sizeof(acadoVariables));

  // Initialize the solver
  acado_initializeSolver();

  // Prepare a consistent initial guess
  for (int i = 0; i < N + 1; ++i) {
    acadoVariables.x[i * NX + 0] = 0;
    acadoVariables.x[i * NX + 1] = 0;
  }

  // Prepare references
  for (int i = 0; i < N; ++i) {
    acadoVariables.y[i * NY + 0] = 1; // theta
    acadoVariables.y[i * NY + 1] = 0; // dtheta
    acadoVariables.y[i * NY + 2] = 0; // T
  }

  acadoVariables.yN[0] = 1; // theta
  acadoVariables.yN[1] = 0; // dtheta
  acadoVariables.yN[2] = 0; // T
}

void ModelPredictiveController::setInitialState(vector<double> x0) {
  for (int i = 0; i < NX; ++i) {
    acadoVariables.x0[i] = x0[i];
  }
}

void ModelPredictiveController::controller(std::vector<double> x0) {

}

