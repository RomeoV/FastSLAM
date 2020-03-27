#ifndef CONFIGFILE_H
#define CONFIGFILE_H

#include <Eigen/Dense>

//******************
// Global Variables
//******************

namespace config {
		extern double V;
		extern double MAXG;
		extern double RATEG;
		extern double WHEELBASE;
		extern double DT_CONTROLS;

		extern double sigmaV;
		extern double sigmaG;

		extern Eigen::Matrix2d Q;

		extern double MAX_RANGE;
		extern double DT_OBSERVE;

		extern double sigmaR;
		extern double sigmaB;
		
		extern Eigen::Matrix2d R;

		extern double AT_WAYPOINT;
		extern int NUMBER_LOOPS;

		extern unsigned int NPARTICLES;
		extern double NEFFECTIVE;

		extern int SWITCH_CONTROL_NOISE;
		extern int SWITCH_SENSOR_NOISE;
		extern int SWITCH_INFLATE_NOISE;
		extern int SWITCH_PREDICT_NOISE;

		extern int SWITCH_SAMPLE_PROPOSAL;
		extern int SWITCH_HEADING_KNOWN;
		extern int SWITCH_RESAMPLE;
		extern int SWITCH_PROFILE;
		extern int SWITCH_SEED_RANDOM;
}
#endif //CONFIGFILE_H
