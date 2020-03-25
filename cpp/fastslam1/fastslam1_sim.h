#ifndef FASTSLAM2_SIM_H
#define FASTSLAM2_SIM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <Eigen/Dense>

#include "core/configfile.h"
#include "core/compute_steering.h"
#include "core/predict_true.h"
#include "core/particle.h"

using namespace std;
using namespace Eigen;

/*!
    Calculates the particles and their positions. Mostly calls other functions.
    @param[out] particles  All particles
    @param[in]  lm        list of landmark data
    @param[in]  wp        list of waypoints, only used to compute steering.
 */
vector<Particle> fastslam1_sim(MatrixXf lm, MatrixXf wp);

/*!
    Makes laser lines (calculate but not used in fastslam1_sim).
    It generates a plot with line_plot_conversion.
    @param[out] matrix of laser lines
    @param[in]  rb        measurements
    @param[in]  xv        robot pose
 */
MatrixXf make_laser_lines(vector<Vector2f> rb, Vector3f xv);

#endif //FASTSLAM2_SIM_H
