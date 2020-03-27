#ifndef COMPUTE_JACOBIANS_H
#define COMPUTE_JACOBIANS_H

#include <Eigen/Dense>
#include "particle.h"
#include <vector>

using namespace std;
using namespace Eigen;

/*!
    Computes the jacobians given a particle state and predict observations. [Compute-Intensive, Switch to mask]
    @param[in]   Particle   Particle for which the jacobian should be computed.
    @param[in]   idf    	Feature indices.
    @param[in]   R        	Covariance matrix of observation (diagonal).
    @param[in]   zp         vector of predicted observation.
    @param[out]  Hv         Jacobian of h wrt vehicle states
    @param[out]  Hf         Jacobian of h wrt feature states
    @param[out]  Sf         Measurement covariance of feature observation given the vehicle.
 */
void compute_jacobians(Particle particle, 
                        vector<int> idf, 
                        Matrix2d R,
                        vector<Vector2d> &zp,
                        vector<Matrix23d> *Hv, 
                        vector<Matrix2d> *Hf, 
                        vector<Matrix2d> *Sf);

#endif //COMPUTE_JACOBIANS_H
