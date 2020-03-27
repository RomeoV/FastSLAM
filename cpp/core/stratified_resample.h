#ifndef STRATIFIED_RESAMPLE_H
#define STRATIFIED_RESAMPLE_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

/*!
    Stratified resampling based on weights w. Interval size is len(w).
    @param[in]   w          Eigen::VectorXd of weights.
	@param[out]  keep       Vector of ints that indicate whether w(i) should be kept or not.
    @param[out]  Neff       = 1/sum(w_i^2).
 */
void stratified_resample(VectorXd w, vector<int> &keep, double &Neff);

/*!
    Returns a vector of all weights below its index are summed up. 
    w_out(i) = sum_j=0->j=i-1(w_in(j))
    @param[out] w   Eigen::VectorXd of weights.
 */
void cumsum(VectorXd &w);

#endif
