#ifndef PARTICLES_H
#define PARTICLES_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

class Particle{
public:
	Particle();
	Particle(float w, Vector3f &xv, Matrix3f &Pv, vector<Vector2f> &xf, vector<Matrix2f> &Pf, float* da);
	~Particle();
        
	//getters	
	float w() const; ///< importance weight
	Vector3f xv() const; ///< robot pose: x,y,theta (heading dir)
	Matrix3f Pv() const; ///< control inputs, i.e. velocities
	vector<Vector2f> xf() const; ///< 2d means of EKF
	vector<Matrix2f> Pf() const; ///< covariance matrices for EKF 
															 ///< length is equal to seen landmarks at time t
	float* da() const; ///< "Not sure, it's never used" (taken from readme)

	//setters
	void setW(float w);
	void setXv(Vector3f &xv);
	void setPv(Matrix3f &Pv);
	void setXf(vector<Vector2f> &xf);
	void setXfi(int i, Vector2f &vec);
	void setPf(vector<Matrix2f> &Pf);
	void setPfi(int i, Matrix2f &m);
	void setDa(float* da);
	
private:
	float _w;
	Vector3f _xv;
	Matrix3f _Pv;		
	vector<Vector2f> _xf;
	vector<Matrix2f> _Pf;
	float* _da;
};

#endif //PARTICLES_H
