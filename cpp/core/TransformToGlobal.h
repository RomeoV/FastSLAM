#ifndef TRANSFORMGLOBAL_H
#define TRANSFORMGLOBAL_H

#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

void TransformToGlobal(MatrixXf &p, Vector3f b);

#endif //TRANSFORMGLOBAL_H
