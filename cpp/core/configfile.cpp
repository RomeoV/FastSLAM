#include "configfile.h"

#define pi 3.14159265

// Configuration file
//Permits various adjustments to parameters of the FastSLAM algorithm.
// See fastslam_sim.h for more information

std::string config::output_filename = "robot_trace.json";

std::string config::ground_truth_filename = "ground_truth.json";

// control parameters
double config::V= 3.0; // m/s
double config::MAXG= 30*pi/180; // radians, maximum steering angle (-MAXG < g < MAXG)
double config::RATEG= 20*pi/180; // rad/s, maximum rate of change in steer angle
double config::WHEELBASE= 4.; // metres, vehicle wheel-base
double config::DT_CONTROLS= 0.025; // seconds, time interval between control signals

// control noises
double config::sigmaV= 0.3; // m/s
double config::sigmaG= (3.0*pi/180); // radians

Eigen::Matrix2d config::Q(2,2);

// observation parameters
double config::MAX_RANGE= 30.0; // metres
double config::DT_OBSERVE= 8* config::DT_CONTROLS; // seconds, time interval between observations

// observation noises
double config::sigmaR= 0.1; // metres
double config::sigmaB= (1.0*pi/180); // radians

Eigen::Matrix2d config::R(2,2);

// waypoint proximity
double config::AT_WAYPOINT= 1.0; // metres, distance from current waypoint at which to switch to next waypoint
int config::NUMBER_LOOPS= 2; // number of loops through the waypoint list

// resampling
unsigned int config::NPARTICLES= 100; 
double config::NEFFECTIVE= 0.75* config::NPARTICLES; // minimum number of effective particles before resampling

// switches
int config::SWITCH_CONTROL_NOISE= 1;
int config::SWITCH_SENSOR_NOISE = 1;
int config::SWITCH_INFLATE_NOISE= 0;
int config::SWITCH_PREDICT_NOISE = 1;   // sample noise from predict (usually 1 for fastslam1.0 and 0 for fastslam2.0)
int config::SWITCH_SAMPLE_PROPOSAL = 1; // sample from proposal (no effect on fastslam1.0 and usually 1 for fastslam2.0)
int config::SWITCH_HEADING_KNOWN= 0;
int config::SWITCH_RESAMPLE= 1; 
int config::SWITCH_PROFILE= 1;
int config::SWITCH_SEED_RANDOM= 0; // if not 0, seed the randn() with its value at beginning of simulation (for repeatability)

