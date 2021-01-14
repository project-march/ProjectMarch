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

ModelPredictiveController::ModelPredictiveController(std::vector<std::vector<float>> Q)
    : Q_(Q)
{
}

void ModelPredictiveController::setReference(vector<vector<double>> reference) {
    for(int i = 0; i < ACADO_N; i++) {
        for(int j = 0; j < ACADO_NY; j++) {
            acadoVariables.y[i*ACADO_NY+j] = reference[i][j];
        }
    }
    for(int j = 0; j < ACADO_NYN; j++) {
        acadoVariables.yN[j] = reference[ACADO_N][j];
    }
    // [Temporary] Has the function been executed?
    std::cout << "\033[4;32m" << __FUNCTION__ << "()\033[0m" << " has executed\n";
}

void ModelPredictiveController::scrollReference(vector<vector<double>>& reference) {
    reference.erase(reference.begin());
    reference.push_back(reference[0]);
    // [Temporary] Has the function been executed?
    std::cout << "\033[4;32m" << __FUNCTION__ << "()\033[0m" << " has executed\n";
}

void ModelPredictiveController::setReference(vector<vector<double>> reference) {
    for(int i = 0; i < ACADO_N; i++) {
        for(int j = 0; j < ACADO_NY; j++) {
            acadoVariables.y[i * ACADO_NY + j] = reference[i][j];
        }
    }
    for(int j = 0; j < ACADO_NYN; j++) {
        acadoVariables.yN[j] = reference[ACADO_N][j];
    }
}

bool ModelPredictiveController::readReferenceFromFile(const char* filename, vector<vector<double>>& data) {
    ifstream file(filename);
    string line;

    if (file.is_open()) {
        while (getline(file,line))
        {
            stringstream linestream(line);
            vector<double> linedata;
            double number;

            while (linestream >> number)
            {
                linedata.push_back(number);
            }
            data.push_back(linedata);
        }
        file.close();
    }
    else
        return false;

    return true;
}

void ModelPredictiveController::setReference(vector<vector<double>>& reference, int iter) {
    for (int i = 0; i < ACADO_N; ++i) {
        acadoVariables.y[i * ACADO_NY + 0] = reference[iter + i][0]; // theta
        acadoVariables.y[i * ACADO_NY + 1] = reference[iter + i][1]; // dtheta
        acadoVariables.y[i * ACADO_NY + 2] = reference[iter + i][2]; // T
    }

    acadoVariables.yN[0] = reference[iter + ACADO_N][0]; // theta
    acadoVariables.yN[1] = reference[iter + ACADO_N][1]; // dtheta
    acadoVariables.yN[2] = reference[iter + ACADO_N][2]; // T
}


void ModelPredictiveController::init() {

  // Read reference data
  if (ModelPredictiveController::readReferenceFromFile("../references/sin.txt", reference) == false) {
      cout << "Cannot read reference" << endl;
  }

  // Initialize the solver
  acado_initializeSolver();

  // Prepare a consistent initial guess
  for (int i = 0; i < ACADO_N + 1; i++) {
    acadoVariables.x[i * ACADO_NX + 0] = 0; // theta
    acadoVariables.x[i * ACADO_NX + 1] = 0; // dtheta
  }

  // Fill reference vector with sinus and or step signals
//  sinRef(reference, 0.2, 0.785, ACADO_N, 0.001);
  stepRef(reference, 0.785, ACADO_N);

  // Set the reference
  setReference(reference);

  // Current state feedback
  setInitialState(x0);

  // Assign the weighting matrix
  assignWeightingMatrix(Q_);

  // Warm-up the solver
  acado_preparationStep();

    // [Temporary] Has the function been executed?
    std::cout << "\033[4;32m" << __FUNCTION__ << "()\033[0m" << " has executed\n";
}

void ModelPredictiveController::setInitialState(vector<double> x0) {
  for (int i = 0; i < ACADO_NX; ++i) {
    acadoVariables.x0[i] = x0[i];
  }
}

void ModelPredictiveController::assignWeightingMatrix(std::vector<std::vector<float>> Q) {

    // Get size of weighting array
    double ACADO_NW = sizeof(acadoVariables.W)/sizeof(acadoVariables.W[0]);
    double ACADO_NWN = sizeof(acadoVariables.WN)/sizeof(acadoVariables.WN[0]);

    // Get size of Q matrix
    int n_rows = Q.size();
    int n_cols = Q[0].size();

    // Check if the given weighting matrix is the correct size.
    // If so, assign the weighting matrices
    if (ACADO_NW == (n_rows*n_cols) && ACADO_NWN == (n_rows-ACADO_NU)*(n_cols-ACADO_NU))
    {

        // set W matrix with Q matrix (state and input weights)
        for(int i=0; i < n_rows; i++) {
            for(int j=0; j < n_cols; j++) {
                acadoVariables.W[i*n_cols+j] = Q[i][j];
            }
        }

        // Set WN matrix with a subset of the Q matrix (only state weights)
        for(int i=0; i < (n_rows-ACADO_NU); i++) {
            for(int j=0; j < 2; j++) {
                acadoVariables.WN[i*(n_cols-ACADO_NU) + j] = Q[i][j];
            }
        }
    }
}

void ModelPredictiveController::calculateControlInput() {

  // Set initial state
  setInitialState(x0);

  // Set reference
  ModelPredictiveController::setReference(reference);
//  ModelPredictiveController::scrollReference(reference);

  // preparation step
  setReference(reference);
  acado_preparationStep();

  // feedback step
  acado_feedbackStep();
  u = acadoVariables.u[0];

  // Shift states and control and prepare for the next iteration
  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  // Scroll the reference vector
  if(repeat_reference) {
      scrollReference(reference);
  }

}
