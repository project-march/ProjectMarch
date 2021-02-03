#ifndef MARCH_MODEL_PREDICTIVE_CONTROLLER_H
#define MARCH_MODEL_PREDICTIVE_CONTROLLER_H

#include <vector>

using namespace std;

class ModelPredictiveController {

public:
    ModelPredictiveController(std::vector<std::vector<float>> Q);

    // Public variables
    vector<double> x0{0,0};             // Current state
    double u;                           // Calculated control input
    vector<vector<double>> reference;   // Current reference
    bool repeat_reference = true;      // Periodically Repeat the reference

    /**
     * \brief Initialise the model predictive controller
     */
    void init();

    /**
    * \brief Set the initial state
    * @param x0 - initial state
    */
    void setInitialState(vector<double> x0);

    /**
     * \brief Set the reference
     * @param reference
     */
    void setReference(vector<vector<double>> reference);

    /**
     * \brief Calculate the control input
     */
    void calculateControlInput();

    /**
     * \brief Assign the weighting matrix values
     * @param Q - weighting matrix
     */
    void assignWeightingMatrix(std::vector<std::vector<float>> Q);

private:
  std::vector<std::vector<float>> Q_;
};

#endif
