#ifndef PARTICLES_H
#define PARTICLES_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

class Particle{
public:
	Particle();
	Particle(float w, VectorXf &xv, MatrixXf &Pv, vector<Vector2f> &xf, vector<MatrixXf> &Pf, float* da);
	~Particle();
        
	//getters	
	float w() const; ///< importance weight
	VectorXf xv() const; ///< robot pose: x,y,theta (heading dir)
	MatrixXf Pv() const; ///< control inputs, i.e. velocities
	vector<Vector2f> xf() const; ///< 2d means of EKF
	vector<MatrixXf> Pf() const; ///< covariance matrices for EKF 
															 ///< length is equal to seen landmarks at time t
	float* da() const; ///< "Not sure, it's never used" (taken from readme)

	//setters
	void setW(float w);
	void setXv(VectorXf &xv);
	void setPv(MatrixXf &Pv);
	void setXf(vector<Vector2f> &xf);
	void setXfi(int i, Vector2f &vec);
	void setPf(vector<MatrixXf> &Pf);
	void setPfi(int i, MatrixXf &m);
	void setDa(float* da);
	
private:
	float _w;
	VectorXf _xv;
	MatrixXf _Pv;		
	vector<Vector2f> _xf;
	vector<MatrixXf> _Pf;
	float* _da;
};

#endif //PARTICLES_H
