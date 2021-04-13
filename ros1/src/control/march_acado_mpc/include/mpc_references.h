#ifndef MARCH_MPC_REFERENCES_H
#define MARCH_MPC_REFERENCES_H

#include <vector>

using namespace std;

/**
 * \brief scroll/progress the reference vector
 * @param reference
 */
void scrollReference(vector<vector<double>>& reference);

/**
 * \brief Add a sinus reference to the existing reference vector with the
 * following parameters
 * @param reference
 * @param freq
 * @param amplitude
 * @param N
 * @param dt
 */
void sinRef(vector<vector<double>>& reference, double freq, double amplitude,
    int N, double dt);

/**
 * \brief Add a step reference to the existing reference vector with the
 * following parameters
 * @param reference
 * @param amplitude
 * @param N
 */
void stepRef(vector<vector<double>>& reference, double amplitude, int N);

#endif // MARCH_MPC_REFERENCES_H
