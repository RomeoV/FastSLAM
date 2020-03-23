#ifndef ADD_OBSERVATION_NOISE_H
#define ADD_OBSERVATION_NOISE_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

namespace nRandMat{
	MatrixXf randn(int m, int n);
	MatrixXf rand(int m, int n); 
}

void add_observation_noise(vector<Vector2f> &z, Matrix2f R, int addnoise);

#endif //ADD_OBSERVATION_NOISE_H
