#include "add_control_noise.h"
#include <iostream>

using namespace std;

void add_control_noise(double V, double G, Matrix2d Q, int addnoise, double* VnGn) 
{
	if (addnoise ==1) {
		Vector2d A(2);
		A(0) = V;
		A(1) = G;
		Vector2d C(2);
		C = multivariate_gauss(A,Q,1);
		VnGn[0] = C(0);
		VnGn[1] = C(1);
	}
}
