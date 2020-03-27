#ifndef SAMPLE_PROPOSAL_H
#define SAMPLE_PROPOSAL_H

#include <vector>
#include <Eigen/Dense>

#include "core/particle.h"
#include "core/compute_jacobians.h"
#include "core/multivariate_gauss.h"
#include "gauss_evaluate.h"
#include "core/pi_to_pi.h"

using namespace Eigen;
using namespace std;

void sample_proposal(Particle &particle, vector<Vector2d> z, vector<int> idf, Matrix2d R);
//double likelihood_given_xv(Particle particle, MatrixXd z, vector<int>idf, MatrixXd R);
double likelihood_given_xv(Particle particle, vector<Vector2d> z, vector<int>idf, Matrix2d R); 
Vector3d delta_xv(Vector3d xv1, Vector3d xv2);

#endif
