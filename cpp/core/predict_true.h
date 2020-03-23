#ifndef PREDICT_TRUE_H
#define PREDICT_TRUE_H

#include <Eigen/Dense>

using namespace Eigen;

/*!
    Clips all angles in angle to range [-pi,pi]. [Simple to optimize]
    @param[out] xv  State vector (x,y,angle).
	@param[in]  V   Velocity in m/s.
    @param[in]  G   Steering angle in radiants.
    @param[in]  WB  Wheelbase.
    @param[in]  dt  timestep.
 */
void predict_true(Vector3f &xv,float V,float G,float WB,float dt); 
	
#endif //PREDICT_TRUE_H
