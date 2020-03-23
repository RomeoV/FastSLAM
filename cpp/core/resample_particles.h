#ifndef RESAMPLE_PARTICLES_H
#define RESAMPLE_PARTICLES_H

#include <Eigen/Dense>
#include <vector>
#include "particle.h"


using namespace Eigen;
using namespace std;

/*!
    Resampling wheel for particles based on their weights (particles.w).
    @param[out] particles   Vector of particles that should be resampled.
	@param[in]  Nmin        Minimum number of particles after resampling.
    @param[in]  doresample  Flag if resampling should be done.
 */
void resample_particles(vector<Particle> &particles, int Nmin, int doresample); 

#endif //RESAMPLE_PARTICLES_H
