#include "KF_joseph_update.h"
#include <iostream>
#include <math.h>

using namespace std;

void KF_joseph_update(Vector3d &x, Matrix3d &P,double v,double R, Matrix13d H)
{
    VectorXd PHt = P*H.transpose();
    MatrixXd S = H*PHt;
    S(0,0) += R;
    MatrixXd Si = S.inverse();
    Si = make_symmetric(Si);
    MatrixXd PSD_check = Si.llt().matrixL(); //chol of scalar is sqrt
    PSD_check.transpose();
    PSD_check.conjugate();

    Vector3d W = PHt*Si;
    x = x+W*v;
    
    //Joseph-form covariance update
    Matrix3d eye(P.rows(), P.cols());
    eye.setIdentity();
    Matrix3d C = eye - W*H;
    P = C*P*C.transpose() + W*R*W.transpose();  

    double eps = 2.2204*pow(10.0,-16); //numerical safety 
    P = P+eye*eps;

    PSD_check = P.llt().matrixL();
    PSD_check.transpose();
    PSD_check.conjugate(); //for upper tri
}

MatrixXd make_symmetric(MatrixXd P)
{
    return (P + P.transpose())*0.5;
}
