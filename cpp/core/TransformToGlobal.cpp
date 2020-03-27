#include "TransformToGlobal.h"
#include "pi_to_pi.h"

void TransformToGlobal(MatrixXd &p, Vector3d b) 
{
	//rotate
	Matrix2d rot(2,2);
	rot<<cos(b(2)), -sin(b(2)), sin(b(2)), cos(b(2));
	
	MatrixXd p_resized;
	p_resized = MatrixXd(p);
	p_resized.conservativeResize(2,p_resized.cols());
	p_resized = rot*p_resized;		
	
	//translate
	int c;
	for (c=0;c<p_resized.cols();c++) {
		p(0,c) = p_resized(0,c)+b(0); 				
		p(1,c) = p_resized(1,c)+b(1); 				
	}

	double input;
	//if p is a pose and not a point
	if (p.rows() ==3){
		for (int k=0; k<p_resized.cols();k++) {
			input = p(2,k) +b(2);
   			p(2,k) = pi_to_pi(input);
		}		
	}	
}
