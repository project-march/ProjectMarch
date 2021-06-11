#ifndef MARCH_MODEL_PREDICTIVE_CONTROLLER_H
#define MARCH_MODEL_PREDICTIVE_CONTROLLER_H

#include <acado_auxiliary_functions.h>
#include <iostream>
#include <vector>

using namespace std;

struct ModelPredictiveControllerConstraints {
    std::vector<float> effort;
    std::vector<float> position_lower;
    std::vector<float> position_upper;
    std::vector<float> velocity;
};

class ModelPredictiveController {

public:
    ModelPredictiveController(
        std::vector<float> W, ModelPredictiveControllerConstraints constraints);

    /**
     * Controller variables
     */

    std::vector<double> command; // calculated input

    /**
     * Diagnostic variables
     */

    // Timing variables
    acado_timer t {};
    double t_preparation {}, t_feedback {};

    // Status variables
    int preparationStepStatus {};
    int feedbackStepStatus {};

    // Performance variables
    double cost {}; // Objective value

    /**
     * \brief Initialise the model predictive controller
     */
    void init();

    /**
     * \brief Set the initial state
     * @param x0 - initial state
     */
    void setInitialState(std::vector<double>& x0);

    /**
     * \brief Set the reference for nodes 0 to N-1
     * @param reference
     */
    void setRunningReference(const std::vector<double>& reference);

    /**
     * \brief Set the reference for node N
     * @param end_reference
     */
    void setEndReference(const std::vector<double>& end_reference);

    /**
     * \brief Assign the weighting array values
     * @param W - weighting array
     */
    void assignWeightingMatrix(std::vector<float> W);

    /**
     * \brief Assign the constraints
     * @param constraints
     */
    void assignConstraints(ModelPredictiveControllerConstraints constraints);

    /**
     * \brief Check status codes and other
     * diagnostic data and issue helpful ROS Messages
     */
    void controllerDiagnosis();

    /**
     * \brief Calculate the control input
     */
    std::vector<double> calculateControlInput();
    /**
     * \brief Shift the state and control acadoVariables
     */
    void shiftStatesAndControl();

private:
    std::vector<float> W_;
    ModelPredictiveControllerConstraints constraints_;
};

#endif
