#ifndef TRANSFORMGLOBAL_H
#define TRANSFORMGLOBAL_H

#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

void TransformToGlobal(MatrixXd &p, Vector3d b);

#endif //TRANSFORMGLOBAL_H
