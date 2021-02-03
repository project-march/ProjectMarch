#ifndef MARCH_MPC_REFERENCES_H
#define MARCH_MPC_REFERENCES_H

#include <vector>

using namespace std;

void scrollReference(vector<vector<double>>& reference);

void sinRef(vector<vector<double>>& reference, double freq, double amp, int N, double dt);

void stepRef(vector<vector<double>>& ref, double amplitude, int N);

#endif //MARCH_MPC_REFERENCES_H
