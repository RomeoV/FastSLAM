#include "compute_weight.h"
#include <Eigen/LU>
#include "core/pi_to_pi.h"
#include "core/compute_jacobians.h"
#include <iostream>
#include <vector>

#define pi 3.1416

using namespace std;
//
//compute particle weight for sampling
//
double compute_weight(Particle &particle, vector<Vector2d> z, vector<int> idf,
	Matrix2d R) 
{
    vector<Matrix23d> Hv;
    vector<Matrix2d> Hf;
    vector<Matrix2d> Sf;
    vector<Vector2d> zp;     

    //process each feature, incrementally refine proposal distribution
    compute_jacobians(particle,idf,R,zp,&Hv,&Hf,&Sf);

    vector<Vector2d> v;
    for (int j =0; j<z.size(); j++) {
	Vector2d v_j = z[j] - zp[j];
	v_j[1] = pi_to_pi(v_j[1]);
	v.push_back(v_j);
    }


    double w = 1.0f;

    Matrix2d S;
    double den, num;
    for (int i=0; i<z.size(); i++) {
	S = Sf[i];
	den = 2*pi*sqrt(S.determinant());
	num = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
	w *= (double)num/(double) den; 
    }

    return w;
} 
