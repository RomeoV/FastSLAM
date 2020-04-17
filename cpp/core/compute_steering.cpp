#include "compute_steering.h"
#include "pi_to_pi.h"

#include <math.h>
#include <vector>
#include <iostream>

using namespace std;


void compute_steering(Vector3d x, MatrixXd wp, int& iwp, double minD, 
				double& G, double rateG, double maxG, double dt)
{

		//determine if current waypoint reached
		Eigen::Vector2d cwp = wp.col(iwp);
		//cwp[0] = wp(0,iwp); //-1 since indexed from 0     
		//cwp[1] = wp(1,iwp);

		double d2 = pow((cwp(0) - x(0)),2) + pow((cwp(1)-x(1)),2); //good    

		if (d2 < minD*minD) { 
				iwp++; //switch to next
				if (iwp >= wp.cols()) {
				    iwp =-1;
				    return;	
				}
				cwp = wp.col(iwp);
				//cwp[0] = wp(0,iwp); //-1 since indexed from 0
				//cwp[1] = wp(1,iwp);
		}

		//compute change in G to point towards current waypoint
		double deltaG = std::atan2(cwp(1)-x(1), cwp(0)-x(0)) - x(2) - G;
		//std::cout<<deltaG<<std::endl;
		deltaG = pi_to_pi(deltaG);
		//std::cout<<deltaG<<std::endl;
		//limit rate
		double maxDelta = rateG*dt;
		if (abs(deltaG) > maxDelta) {
				int sign = (deltaG > 0) ? 1 : ((deltaG < 0) ? -1 : 0);
				deltaG = sign*maxDelta;	
		}	

		//limit angle
		G = G+deltaG;
		if (abs(G) > maxG) {
				int sign2 = (G > 0) ? 1: ((G < 0) ? -1 : 0);
				G = sign2*maxG;
		}
}
