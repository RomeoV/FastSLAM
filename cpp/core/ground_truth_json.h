#pragma once
#include <iostream>
#include <array>
#include <vector>
#include "core/json.hpp"
#include <Eigen/Dense>



nlohmann::json ground_truth_step_json(Eigen::Vector3d xtrue, double time, int id, int iwp, double G);

nlohmann::json ground_truth_keypoints_json(Eigen::MatrixXd& waypoints, Eigen::MatrixXd& landmarks);