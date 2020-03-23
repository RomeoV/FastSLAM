#ifndef ADD_OBSERVATION_NOISE_H
#define ADD_OBSERVATION_NOISE_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

namespace nRandMat{
	MatrixXf randn(int m, int n); //Gaussian distribution
	MatrixXf rand(int m, int n); //Standard random
}

/*!
    Adds random observation noise to the observation vector. [Compute intensive]
    @param[out] z        	Landmark measurements / observations [meter, radians].
    @param[in]  R        	Covariance matrix of observation (diagonal).
    @param[in]  addnoise	Flag if obersvation noise should be added.
 */
void add_observation_noise(vector<Vector2f> &z, Matrix2f R, int addnoise);

#endif //ADD_OBSERVATION_NOISE_H
