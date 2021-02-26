#include "mpc_references.h"

#include <vector>
#include <cmath>

using namespace std;

void scrollReference(vector<vector<double>>& reference) {
    reference.erase(reference.begin());
    reference.push_back(reference[0]);
}

void sinRef(vector<vector<double>>& reference, double freq, double amplitude, int N, double dt) {

    // calculate the amount of time steps required for one period
    double T_N = ceil(1/(freq*dt));

    // multiply the required time steps with the amount of required periods to have at least N reference values
    T_N = T_N*ceil(N/T_N)+1;

    vector<double> line;
    for (int j = 0; j < T_N; j++) {
        // clear the line vector
        line.clear();

        // fill the line vector with the three required references values
        line.push_back(amplitude*sin((freq*2*M_PI)*(j*dt))); // THETA
        line.push_back(amplitude*cos((freq*2*M_PI)*(j*dt))); // DTHETA
        line.push_back(0.0); // T

        // add the temp vector to the reference
        reference.push_back(line);
    }

}

void stepRef(vector<vector<double>>& reference, double amplitude, int N) {

    vector<double> line;
    for (int j = 0; j < N+1; j++) {
        // clear the line vector
        line.clear();

        // fill the line vector with the three required references values
        line.push_back(amplitude);  // THETA
        line.push_back(0.0);        // DTHETA
        line.push_back(0.0);        // T

        // add the temp vector to the reference
        reference.push_back(line);
    }

}