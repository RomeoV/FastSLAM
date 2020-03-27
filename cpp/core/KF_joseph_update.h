#ifndef KF_JOSEPH_UPDATE_H
#define KF_JOSEPH_UPDATE_H

#include <Eigen/Dense>
#include "particle.h"

using namespace Eigen;

void KF_joseph_update(Vector3d &x,Matrix3d &P,double v,double R, Matrix13d H);
MatrixXd make_symmetric(MatrixXd P);

#endif
