#ifndef ADD_CONTROL_NOISE
#define ADD_CONTROL_NOISE

#include <Eigen/Dense>
#include "multivariate_gauss.h"
# include <cstdlib>
# include <cmath>

using namespace Eigen;

/*****************************************************************************
 * IMPLEMENTATION STATUS
 * Last Worked on: 23.03.2020
 * Done: Added docstring
 * ToDo: Rewrite for C implementation
 * C Implementation done: No
 ****************************************************************************/

/*****************************************************************************
 * PERFORMANCE STATUS
 * Work:
 * Memory moved:
 * Cycles:
 * Performance:
 * Optimal:
 * Status:
 ****************************************************************************/

/*!
    Adds control noise (for velocity and steering angle) to the model [Hardly 
    any optimizations possible].
    @param[out] VnGn     Array where VnGn = [V+noise, G+noise].
    @param[in]  V        Speed [m/s] -> see configfile.h
    @param[in]  G        Steering angle.
    @param[in]  Q        Covariance matrix of V and G.
    @param[in]  addnoise Flag if control noise should be added.
 */
void add_control_noise(double V, double G, Matrix2d Q, int addnoise,double* VnGn);


#endif //ADD_CONTROL_NOISE
