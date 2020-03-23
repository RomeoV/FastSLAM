#ifndef KF_JOSEPH_UPDATE_H
#define KF_JOSEPH_UPDATE_H

#include <Eigen/Dense>
#include "particle.h"

using namespace Eigen;

void KF_joseph_update(Vector3f &x,Matrix3f &P,float v,float R, Matrix13f H);
MatrixXf make_symmetric(MatrixXf P);

#endif
