#include "pi_to_pi.h"
#include <iostream>


/*!
    Clips all angles in angle (radiants) to range [-pi,pi].
	@param[out] angle   Vector of angles (radiants).
 */
void pi_to_pi(VectorXd &angle) 
{
    int n;
    for (int i=0; i<angle.size(); i++) {
        if ((angle[i] < (-2*pi)) || (angle[i] > (2*pi))) {
            n=floor(angle[i]/(2*pi));
            angle[i] = angle[i]-n*(2*pi);    
            
            if (angle[i] > pi) {
                angle[i] = angle[i] - (2*pi);
            }
            if (angle[i] < -pi) {
                angle[i] = angle[i] + (2*pi);
            }
        }
    }
}

/*!
    Clips ang  to range [-pi,pi].
	@param[out] angle   angle in radiants.
 */
double pi_to_pi(double ang) 
{
    int n;
    if ((ang < (-2*pi)) || (ang > (2*pi))) {
        n=floor(ang/(2*pi));
        ang = ang-n*(2*pi);    
    }
    if (ang > pi) {
        ang = ang - (2*pi);
    }
    if (ang < -pi) {
        ang = ang + (2*pi);
    }
    return ang;
}

