#include "multivariate_gauss.h"
#include "add_observation_noise.h"
#include <Eigen/Cholesky>
#include <iostream>

using namespace std;
//x is mean vector
//P is covariance matrix
//n obtain n samples

//output: sample set

VectorXd multivariate_gauss(VectorXd x, MatrixXd P, int n) 
{
	int len = x.size();
	//choleksy decomposition
	MatrixXd S = P.llt().matrixL();
	MatrixXd X(len,n);
	

    double LO = -1.0f;
    double HI = 1.0f;

    for (int i = 0; i < len; i++) {
        for (int j=0; j< n; j++) {
            double r3 = LO + (double)rand()/((double)RAND_MAX/(HI-LO));
            X(i,j) = r3;
        }
    }

    //TODO: this does not work. Also fixed other instances of nRandMat	
	//X = nRandMat::randn(len,n);	
	
	MatrixXd ones = MatrixXd::Ones(1,n);	
	return S*X + x*ones;
}

