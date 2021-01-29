#ifndef MARCH_MODEL_PREDICTIVE_CONTROLLER_H
#define MARCH_MODEL_PREDICTIVE_CONTROLLER_H

#include <vector>

using namespace std;

class ModelPredictiveController {

public:
    // Public variables
    vector<double> x0{0,0}; // Current state
    double u;               // Calculated control input
    vector<vector<double>> reference; // Current reference

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
     * \brief Calculate the control input
     * @param x0 - initial state
     * @return u - control input
     */
    void calculateControlInput();

    void sinRef(vector<vector<double>>& ref, double freq, double amp, int N, double dt);

    void stepRef(vector<vector<double>>& ref, double amp, int N, double dt);

    void scrollReference(vector<vector<double>>& reference);

    void setReference(vector<vector<double>> reference);
};

#endif
