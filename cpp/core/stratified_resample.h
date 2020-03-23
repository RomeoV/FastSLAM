#ifndef STRATIFIED_RESAMPLE_H
#define STRATIFIED_RESAMPLE_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

/*!
    Stratified resampling based on weights w. Interval size is len(w).
    @param[in]   w          Eigen::VectorXf of weights.
	@param[out]  keep       Vector of ints that indicate whether w(i) should be kept or not.
    @param[out]  Neff       = 1/sum(w_i^2).
 */
void stratified_resample(VectorXf w, vector<int> &keep, float &Neff);

/*!
    Returns a vector of all weights below its index are summed up. 
    w_out(i) = sum_j=0->j=i-1(w_in(j))
    @param[out] w   Eigen::VectorXf of weights.
 */
void cumsum(VectorXf &w);

#endif
