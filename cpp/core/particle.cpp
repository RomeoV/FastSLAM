#include <iostream>
#include "particle.h"
#include <algorithm>

/*************
** Particle
*************/

Particle::Particle() 
{
	_w = 1.0; 
	_xv = VectorXf(3);
        _xv.setZero();
	_Pv = Matrix3f(3,3);
        _Pv.setZero();
	_da = NULL;
}

Particle::Particle(float w, Vector3f &xv, Matrix3f &Pv, vector<Vector2f> &xf, vector<Matrix2f> &Pf, float* da)
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
float Particle::w() const
{
	return _w;	
}

Vector3f Particle::xv() const
{
	return _xv;	
}

Matrix3f Particle::Pv() const
{
	return _Pv;	
}

vector<Vector2f> Particle::xf() const
{
	return _xf;	
}

vector<Matrix2f> Particle::Pf() const
{
	return _Pf;	
}

float* Particle::da() const
{
	return _da;	
}

//setters
void Particle::setW(float w)
{
	_w = w;
}

void Particle::setXv(Vector3f &xv)
{
	_xv = xv;
}

void Particle::setPv(Matrix3f &Pv)
{
	_Pv = Pv;
}

void Particle::setXf(vector<Vector2f> &xf)
{
	_xf = xf;
}

void Particle::setXfi(int i, Vector2f &vec) 
{
	if (i >= _xf.size()){
		_xf.resize(i+1);
	}
	_xf[i] = vec;
}

void Particle::setPf(vector<Matrix2f> &Pf)
{
	_Pf = Pf;
}

void Particle::setPfi(int i, Matrix2f &m) 
{
	if(i >= _Pf.size()) {
		_Pf.resize(i+1);
	}
	_Pf[i] = m;
}

void Particle::setDa(float* da)
{
	_da = da;
}

