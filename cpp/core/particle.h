#ifndef PARTICLES_H
#define PARTICLES_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

namespace Eigen {
	using Matrix13d = Matrix<double, 1, 3>;
	using Matrix31d = Matrix<double, 3, 1>;
	using Matrix23d = Matrix<double, 2, 3>;
	using Matrix32d = Matrix<double, 3, 2>;
}

/*!
	We should rewrite this as a struct, maybe preserving some temporal locality.
	All std::vector could be replaced with masks. 
	
*/
class Particle{
public:
	Particle();
	Particle(double w, Vector3d &xv, Matrix3d &Pv, vector<Vector2d> &xf, vector<Matrix2d> &Pf, double* da);
	~Particle();
        
	//getters	
	double w() const; ///< importance weight
	Vector3d xv() const; ///< robot pose: x,y,theta (heading dir)
	Matrix3d Pv() const; ///< control inputs, i.e. velocities
	vector<Vector2d> xf() const; ///< 2d means of EKF
	vector<Matrix2d> Pf() const; ///< covariance matrices for EKF 
															 ///< length is equal to seen landmarks at time t
	double* da() const; ///< "Not sure, it's never used" (taken from readme)

	//setters
	void setW(double w);
	void setXv(Vector3d &xv);
	void setPv(Matrix3d &Pv);
	void setXd(vector<Vector2d> &xf);
	void setXdi(int i, Vector2d &vec);
	void setPf(vector<Matrix2d> &Pf);
	void setPfi(int i, Matrix2d &m);
	void setDa(double* da);
	
private:
	double _w;
	Vector3d _xv;
	Matrix3d _Pv;		
	vector<Vector2d> _xf;
	vector<Matrix2d> _Pf;
	double* _da;
};

#endif //PARTICLES_H
