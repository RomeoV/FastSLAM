#include "KF_joseph_update.h"
#include <iostream>

using namespace std;

//TODO: check this!
void KF_joseph_update(VectorXf &x, MatrixXf &P,float v,float R, MatrixXf H)
{
    VectorXf PHt = P*H.transpose();
    
	MatrixXf S = H*PHt;
	S(0,0) += R;
    MatrixXf Si = S.inverse();	
    Si = make_symmetric(Si); 
    MatrixXf PSD_check = Si.llt().matrixL(); //chol of scalar is sqrt
    PSD_check.transpose();
    PSD_check.conjugate();

    VectorXf W = PHt*Si;
    x = x+W*v;
    
    //Joseph-form covariance update
    MatrixXf eye(P.rows(), P.cols());
    eye.setIdentity();
    MatrixXf C = eye - W*H;
    P = C*P*C.transpose() + W*R*W.transpose();  
   
    float eps = 2.2204*pow(10,-16); //numerical safety 
    P = P+eye*eps;
	cout<<"in KF_joseph_update, P"<<endl;
	cout<<P<<endl;
	
    PSD_check = P.llt().matrixL();
    PSD_check.transpose();
    PSD_check.conjugate(); //for upper tri
}

MatrixXf make_symmetric(MatrixXf P)
{
    return (P + P.transpose())*0.5;
}
