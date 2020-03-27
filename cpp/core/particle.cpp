#include <iostream>
#include "particle.h"
#include <algorithm>

/*************
** Particle
*************/

Particle::Particle() 
{
	_w = 1.0; 
	_xv = Vector3d(3);
        _xv.setZero();
	_Pv = Matrix3d(3,3);
        _Pv.setZero();
	_da = NULL;
}

Particle::Particle(double w, Vector3d &xv, Matrix3d &Pv, vector<Vector2d> &xf, vector<Matrix2d> &Pf, double* da)
{
	_w = w;
	_xv = xv;
	_Pv = Pv;
	_xf = xf;
	_Pf = Pf;
	_da = da;		
}

Particle::~Particle()
{
}

//getters
double Particle::w() const
{
	return _w;	
}

Vector3d Particle::xv() const
{
	return _xv;	
}

Matrix3d Particle::Pv() const
{
	return _Pv;	
}

vector<Vector2d> Particle::xf() const
{
	return _xf;	
}

vector<Matrix2d> Particle::Pf() const
{
	return _Pf;	
}

double* Particle::da() const
{
	return _da;	
}

//setters
void Particle::setW(double w)
{
	_w = w;
}

void Particle::setXv(Vector3d &xv)
{
	_xv = xv;
}

void Particle::setPv(Matrix3d &Pv)
{
	_Pv = Pv;
}

void Particle::setXd(vector<Vector2d> &xf)
{
	_xf = xf;
}

void Particle::setXdi(int i, Vector2d &vec) 
{
	if (i >= _xf.size()){
		_xf.resize(i+1);
	}
	_xf[i] = vec;
}

void Particle::setPf(vector<Matrix2d> &Pf)
{
	_Pf = Pf;
}

void Particle::setPfi(int i, Matrix2d &m) 
{
	if(i >= _Pf.size()) {
		_Pf.resize(i+1);
	}
	_Pf[i] = m;
}

void Particle::setDa(double* da)
{
	_da = da;
}

