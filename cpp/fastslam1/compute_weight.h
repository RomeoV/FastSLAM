#ifndef COMPUTE_WEIGHT_H
#define COMPTUE_WEIGHT_H

#include <Eigen/Dense>
#include <vector>
#include "core/particle.h"

using namespace Eigen;
using namespace std;

double compute_weight(Particle &particle, vector<Vector2d> z, vector<int> idf, Matrix2d R); 

#endif
