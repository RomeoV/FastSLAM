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
float compute_weight(Particle &particle, vector<Vector2f> z, vector<int> idf,
	Matrix2f R) 
{
    vector<Matrix23f> Hv;
    vector<Matrix2f> Hf;
    vector<Matrix2f> Sf;
    vector<Vector2f> zp;     

    //process each feature, incrementally refine proposal distribution
    compute_jacobians(particle,idf,R,zp,&Hv,&Hf,&Sf);

    vector<Vector2f> v;
    for (int j =0; j<z.size(); j++) {
	Vector2f v_j = z[j] - zp[j];
	v_j[1] = pi_to_pi(v_j[1]);
	v.push_back(v_j);
    }


    float w = 1.0f;

    Matrix2f S;
    float den, num;
    for (int i=0; i<z.size(); i++) {
	S = Sf[i];
	den = 2*pi*sqrt(S.determinant());
	num = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
	w *= (float)num/(float) den; 
    }

    return w;
} 
