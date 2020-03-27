#ifndef FEATURE_UPDATE_H
#define FEATURE_UPDATE_H

#include <Eigen/Dense>
#include <vector>

#include "particle.h"
#include "compute_jacobians.h"
#include "pi_to_pi.h"
#include "KF_cholesky_update.h"

using namespace Eigen;
using namespace std;

/*!
    Updates the state of a particle given a list of measurements. [Runtime depends
     on pi_to_pi and KF_cholesky, otherwise mostly memory operations. Switch to mask!]
	@param[out] 	particle 	Particle to be updated.
	@param[in] 		z		    list of measurements conditioned on the particle.
	@param[out] 	idf 	    Index of known landmarks.
	@param[out] 	R	 	    Covariance Matrix of measurements.
 */
void feature_update(Particle &particle, vector<Vector2d> z, vector<int>idf, Matrix2d R);

#endif
