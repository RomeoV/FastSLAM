#include "sample_proposal.h"
#include <iostream>
#include <Eigen/SVD>
#include <iomanip>
#include <math.h>
#include "assert.h"

//compute proposal distribution, then sample from it, and compute new particle weight
void sample_proposal(Particle &particle, vector<Vector2d> z, vector<int> idf, Matrix2d R)
{
    assert(isfinite(particle.w()));
    Vector3d xv = Vector3d(particle.xv()); //robot position
    Matrix3d Pv = Matrix3d(particle.Pv()); //controls (motion command)

    Vector3d xv0 = Vector3d(xv);
    Matrix3d Pv0 = Matrix3d(Pv);	

    vector<Matrix23d> Hv;
    vector<Matrix2d> Hf;
    vector<Matrix2d> Sf;
    vector<Vector2d> zp;

    Vector2d zpi;
    Matrix23d Hvi;
    Matrix2d Hfi;
    Matrix2d Sfi;

    //process each feature, incrementally refine proposal distribution
    unsigned i,r;
    vector<int> j;
    for (i =0; i<idf.size(); i++) {
        j.clear();
        j.push_back(idf[i]);

        Hv.clear();
        Hf.clear();
        Sf.clear();
        zp.clear();

        compute_jacobians(particle,j,R,zp,&Hv,&Hf,&Sf);

        zpi = zp[0];
        Hvi = Hv[0];
        Hfi = Hf[0];
        Sfi = Sf[0];

        Vector2d vi = z[i] - zpi;
        vi[1] = pi_to_pi(vi[1]);

        //proposal covariance
        Pv = Hvi.transpose() * Sfi * Hvi + Pv.inverse();
        Pv = Pv.inverse().eval();

        //proposal mean
        xv = xv + Pv * Hvi.transpose() * Sfi * vi;
        particle.setXv(xv);
        particle.setPv(Pv); 
    }

    //sample from proposal distribution
    Vector3d xvs = multivariate_gauss(xv,Pv,1); 
    particle.setXv(xvs);
    Matrix3d zeros(3,3);
    zeros.setZero();
    particle.setPv(zeros);

    //compute sample weight: w = w* p(z|xk) p(xk|xk-1) / proposal
    double like = likelihood_given_xv(particle, z, idf, R);
    double prior = gauss_evaluate(delta_xv(xv0,xvs), Pv0,0);
    double prop = gauss_evaluate(delta_xv(xv,xvs),Pv,0);
    assert(isfinite(particle.w()));

    double a = prior/prop;
    double b = particle.w() * a;
    double newW = like * b;
    //double newW = particle.w() * like * prior / prop;
    #if 0
    if (!isfinite(newW)) {
	cout<<"LIKELIHOOD GIVEN XV INPUTS"<<endl;   
	cout<<"particle.w()"<<endl;
	cout<<particle.w()<<endl;
	cout<<"particle.xv()"<<endl;
	cout<<particle.xv()<<endl;
	
	cout<<"particle.Pv()"<<endl;
	cout<<particle.Pv()<<endl;
	cout<<"particle.xf"<<endl;
	for (int i =0; i<particle.xf().size(); i++) {
	    cout<<particle.xf()[i]<<endl;
	}
	cout<<endl;
	cout<<"particle.Pf()"<<endl;
	for (int i =0; i< particle.Pf().size(); i++) {
	    cout<<particle.Pf()[i]<<endl;
	}
	cout<<endl;
	
	cout<<"z"<<endl;
	for (int i=0; i<z.size(); i++) {
	    cout<<z[i]<<endl;
	}   
	cout<<endl;
	
	cout<<"idf"<<endl;
	for (int i =0; i<idf.size(); i++){
	    cout<<idf[i]<<" ";
	}		
	cout<<endl;

	cout<<"R"<<endl;
	cout<<R<<endl;

	cout<<"GAUSS EVALUATE INPUTS"<<endl;
	cout<<"delta_xv(xv0,xvs)"<<endl;	
	cout<<delta_xv(xv0,xvs)<<endl;
	
	cout<<"Pv0"<<endl;
	cout<<Pv0<<endl;
	
	cout<<"delta_xv(xv,xvs)"<<endl;	
	cout<<delta_xv(xv,xvs)<<endl;
	
	cout<<"Pv"<<endl;
	cout<<Pv<<endl;
		
	
	cout<<"like"<<endl;
	cout<<like<<endl;
	cout<<"prior"<<endl;
	cout<<prior<<endl;
	cout<<"prop"<<endl;
	cout<<prop<<endl;
    }
    #endif
    particle.setW(newW);
} 

double likelihood_given_xv(Particle particle, vector<Vector2d> z, vector<int>idf, Matrix2d R) 
{
    double w = 1;

    vector<Matrix23d> Hv;
    vector<Matrix2d> Hf;
    vector<Matrix2d> Sf;
    vector<Vector2d> zp;

    unsigned j,k;
    vector<int> idfi;
    
    for (j=0; j<idf.size(); j++){
	idfi.clear();
        idfi.push_back(idf[j]);
	Hv.clear();
	Hf.clear();
	Sf.clear();
	zp.clear();

	Vector2d v(z.size()); 
        compute_jacobians(particle,idfi,R,zp,&Hv,&Hf,&Sf);
	v = z[j] - zp[0];
        v(1) = pi_to_pi(v(1));
        w = w*gauss_evaluate(v,Sf[0],0);
    } 
    return w;
}

Vector3d delta_xv(Vector3d xv1, Vector3d xv2)
{
    //compute innovation between two xv estimates, normalising the heading component
    Vector3d dx = xv1-xv2; 
    dx(2) = pi_to_pi(dx(2));
    return dx;
}
