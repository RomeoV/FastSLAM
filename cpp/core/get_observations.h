#ifndef GET_OBSERVATIONS_H
#define GET_OBSERVATIONS_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

vector<Vector2f> get_observations(Vector3f x,MatrixXf lm,vector<int> &idf,float rmax);
void get_visible_landmarks(Vector3f x, MatrixXf &lm,vector<int> &idf, float rmax);
vector<Vector2f> compute_range_bearing(Vector3f x, MatrixXf lm);
vector<int> find2(vector<float> dx, vector<float> dy, float phi, float rmax);

#endif
