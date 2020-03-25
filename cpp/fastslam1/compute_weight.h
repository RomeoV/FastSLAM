#ifndef COMPUTE_WEIGHT_H
#define COMPTUE_WEIGHT_H

#include <Eigen/Dense>
#include <vector>
#include "core/particle.h"

using namespace Eigen;
using namespace std;

/*!
    Compute particle weight for sampling.
    Uses compute_jacobians.
    @param[out] particle Particle whose weight is calculated
    @param[in]  z        vector of map features, calculated by data_associate_known
    @param[in]  idf      vector of map features, calculated by data_associate_known, used for jacobians 
    @param[in]  R        matrix of observation noises, metres and radians
 */
float compute_weight(Particle &particle, vector<Vector2f> z, vector<int> idf, Matrix2f R); 

#endif
