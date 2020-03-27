#include "predict.h"
#include <math.h>
#include <iostream>

#define pi 3.1416

using namespace std;

void predict(Particle &particle,double V,double G,Matrix2d Q, double WB,double dt, int addrandom)
{
	//optional: add random noise to predicted state
	if (addrandom ==1) {
		Vector2d A(2);
		A(0) = V;
		A(1) = G;
		Vector2d VG(2);
		VG = multivariate_gauss(A,Q,1);	
		V = VG(0);
		G = VG(1);	
	}	

	//predict state
	Vector3d xv = particle.xv();
	Vector3d xv_temp(3);
	xv_temp << xv(0) + V*dt*cos(G+xv(2)),
			xv(1) + V*dt*sin(G+xv(2)),
			pi_to_pi2(xv(2) + V*dt*sin(G/WB));
	particle.setXv(xv_temp);
}

double pi_to_pi2(double ang) 
{
	if (ang > pi) {
		ang = ang - (2*pi);
	}
	if (ang < -pi) {
		ang = ang + (2*pi);
	}
	return ang;
}

