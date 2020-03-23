#ifndef MULTIVARIATE_GAUSS_H
#define MULTIVARIATE_GAUSS_H

#include <Eigen/Dense>

using namespace Eigen;

/*!
    Computes Multivariate Gaussian for n Samples. [Pretty simple adaptions, usually x is size 2 and P is 2x2]
    @param[out]          Sample set. Size len(x) x n (not necessarily a vector if n!=1)
    @param[in]  x        Mean vector (e.g. (V,G)).
    @param[in]  P        Covariance Matrix.
    @param[in]  n        Number of samples.
 */
VectorXf multivariate_gauss(VectorXf x, MatrixXf P, int n);

#endif //MULTIVARIATE_GAUSS_H

