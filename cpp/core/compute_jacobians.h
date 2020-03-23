#ifndef COMPUTE_JACOBIANS_H
#define COMPUTE_JACOBIANS_H

#include <Eigen/Dense>
#include "particle.h"
#include <vector>

using namespace std;
using namespace Eigen;

void compute_jacobians(Particle particle, 
                        vector<int> idf, 
                        Matrix2f R,
                        vector<Vector2f> &zp,
                        vector<Matrix23f> *Hv, 
                        vector<Matrix2f> *Hf, 
                        vector<Matrix2f> *Sf);

#endif //COMPUTE_JACOBIANS_H
