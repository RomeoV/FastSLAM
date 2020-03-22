#ifndef KF_CHOLESKY_UPDATE_H
#define KF_CHOLESKY_UPDATE_H

#include <Eigen/Dense>

using namespace Eigen;

void KF_cholesky_update(Vector2f &x,Matrix2f &P,VectorXf v,MatrixXf R,MatrixXf H);

#endif
