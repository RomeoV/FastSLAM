#ifndef STRATIFIED_RANDOM_H
#define STRATIFIED_RANDOM_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

/*!
    Stratified random vector (Sampling with subpopulations).
    @param[in]  N     Interval size (inverse).
	@param[out] di    Generated array of random stratified numbers.
 */
void stratified_random(int N, vector<float> &di);

/*!
    Generates a random, uniformly sampled number between [0,1].
    @return random uniform number in [0,1].
 */
double unifRand();

#endif
