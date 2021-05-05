#define _USE_MATH_DEFINES

#include "model_predictive_controller.hpp"
#include "acado_auxiliary_functions.h"
#include "acado_common.h"
#include "acado_qpoases_interface.hpp"
#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// Global variables used by the solver
ACADOvariables acadoVariables = {};
ACADOworkspace acadoWorkspace = {};

ModelPredictiveController::ModelPredictiveController(std::vector<float> W)
    : W_(std::move(W))
{
}

void ModelPredictiveController::init()
{
    // Initialize the solver
    acado_initializeSolver();

    // Initialize state array with zero
    std::fill(std::begin(acadoVariables.x), std::end(acadoVariables.x), 0.0);

    // Initialize input array with zero
    std::fill(std::begin(acadoVariables.u), std::end(acadoVariables.u), 0.0);

    // Initialize "running" and "end" reference array with zero
    std::fill(std::begin(acadoVariables.y), std::end(acadoVariables.y), 0.0);
    std::fill(std::begin(acadoVariables.yN), std::end(acadoVariables.yN), 0.0);

    // Initialize "running" and "end" weighting array with zero
    std::fill(std::begin(acadoVariables.W), std::end(acadoVariables.W), 0.0);
    std::fill(std::begin(acadoVariables.WN), std::end(acadoVariables.WN), 0.0);

    // Reserve space for the inputs
    command.reserve(ACADO_NU);

    // Assign the weighting matrix
    assignWeightingMatrix(W_);

    // Warm-up the solver
    acado_preparationStep();
}

void ModelPredictiveController::setInitialState(std::vector<double>& x0)
{
    std::copy(x0.begin(), x0.end(), std::begin(acadoVariables.x0));
}

void ModelPredictiveController::setRunningReference(
    const std::vector<double>& reference)
{
    // check if size of reference is equal to the size of acadoVariables.y
    if (reference.size()
        != sizeof(acadoVariables.y) / sizeof(acadoVariables.y[0])) {
        ROS_WARN_STREAM_ONCE(
            "The \"running\" reference vector has an incorrect size");
    }

    // copy the reference vector to the acadoVariables.y array
    std::copy(reference.begin(), reference.end(), std::begin(acadoVariables.y));
}

void ModelPredictiveController::setEndReference(
    const std::vector<double>& end_reference)
{
    // check if size of end_reference is equal to the size of acadoVariables.yN
    if (end_reference.size() != ACADO_NYN) {
        ROS_WARN_STREAM_ONCE(
            "The \"end\" reference vector has an incorrect size");
    }

    // copy the end_reference vector to the acadoVariables.yN array
    std::copy(end_reference.begin(), end_reference.end(),
        std::begin(acadoVariables.yN));
}

void ModelPredictiveController::shiftStatesAndControl()
{
    acado_shiftStates(/*strategy=*/2, /*xEnd=*/nullptr, /*uEnd=*/nullptr);
    acado_shiftControls(/*uEnd=*/nullptr);
}

void ModelPredictiveController::assignWeightingMatrix(std::vector<float> W)
{
    // set the diagonal of the ACADO W matrix (state and input weights)
    for (int i = 0; i < ACADO_NY; ++i) {
        acadoVariables.W[i * (ACADO_NY + 1)] = W[i];
    }

    // Set the diagonal of the ACADO WN matrix (only state weights)
    for (int i = 0; i < ACADO_NYN; ++i) {
        acadoVariables.WN[i * (ACADO_NYN + 1)] = W[i];
    }
}

void ModelPredictiveController::controllerDiagnosis()
{
    // Check acado_preparationStep() status code
    ROS_WARN_STREAM_COND(preparationStepStatus != 0,
        "MPC_PREP, " << acado_getErrorString(preparationStepStatus));

    // Check acado_feedbackStep() status code
    ROS_WARN_STREAM_COND(feedbackStepStatus != 0,
        "MPC_FEEDBACK, " << acado_getErrorString(feedbackStepStatus));
}

std::vector<double> ModelPredictiveController::calculateControlInput()
{
    // Preparation step (timed)
    acado_tic(&t);
    preparationStepStatus = acado_preparationStep();
    t_preparation = acado_toc(&t);

    // Feedback step (timed)
    acado_tic(&t);
    feedbackStepStatus = acado_feedbackStep();
    t_feedback = acado_toc(&t);

    // Objective cost for diagnosis
    cost = acado_getObjective();

    // Perform a diagnosis on the controller
    controllerDiagnosis();

    // get command
    command.assign(
        std::begin(acadoVariables.u), std::begin(acadoVariables.u) + ACADO_NU);

    // return command
    return command;
}
