#ifndef PREDICT_H
#define PREDICT_H

#include <Eigen/Dense>
#include "core/particle.h"
#include "core/multivariate_gauss.h"

using namespace Eigen;

void predict(Particle &particle,double V,double G,Matrix2d Q, double WB,double dt, int addrandom);
double pi_to_pi2(double ang); 

#endif //PREDICT_H
