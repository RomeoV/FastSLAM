#ifndef ADD_FEATURE_H
#define ADD_FEATURE_H

#include <Eigen/Dense>

#include "particle.h"

using namespace Eigen;


/*!
    Adds a feature observation to a particle. [Memory-heavy, leave for later, but switch to mask]
    @param[out] particle Particle where the feature is added to.
    @param[in]  z        Landmark measurements / observations [meter, radians].
    @param[in]  R        Covariance matrix of observation noises -> configfile.h
 */
void add_feature(Particle &particle, vector<Vector2f> z, Matrix2f R);
 
#endif //ADD_FEATURE_H

