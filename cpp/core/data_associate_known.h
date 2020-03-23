#ifndef DATA_ASSOCIATE_KNOWN_H
#define DATA_ASSOCIATE_KNOWN_H

#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

void data_associate_known(vector<Vector2f> z, vector<int> idz, VectorXf &table, int Nf, \
						  vector<Vector2f> &zf, vector<int> &idf, vector<Vector2f> &zn); 

#endif //DATA_ASSOCIATE_KNOWN_H
