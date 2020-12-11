#include "model_predictive_controller.hpp"
#include <vector>

#include <iostream>
using namespace std;

void ModelPredictiveController::initSolver() {
  cout << "ModelPredictiveController" << endl;
}

void ModelPredictiveController::setInitialState(vector<double> x0) {
  for (int i = 0; i < NX; ++i) {
    acadoVariables.x0[i] = x0[i];
  }
}
