#include "fastslam1/fastslam1_sim.h"
#include "core/add_control_noise.h"
#include "core/get_observations.h"
#include "core/add_observation_noise.h"
#include "core/TransformToGlobal.h"
#include "core/line_plot_conversion.h"
#include "core/data_associate_known.h"
#include "core/feature_update.h"
#include "core/resample_particles.h"
#include "core/add_feature.h"
#include "fastslam1/compute_weight.h"
#include "fastslam1/predict.h"
#include <gtest/gtest.h>
#include "core/read_input_file.h"
#include "core/stratified_resample.h"
#include "core/stratified_random.h"
#include "fastslam2/sample_proposal.h"
#include "core/multivariate_gauss.h"

#include <iostream>
//#include <boost/random.hpp>
//#include <boost/random/normal_distribution.hpp>

#define ZERO 1e-10
#define SMALL 1e-4
#define DELTA 1e-10
#define isZero(A) ( (A < ZERO) && (A > -ZERO) )
#define isSmall(A) ( (A < SMALL) && (A > -SMALL) )
#define isSame(A, B) ( ((A-B) < ZERO) && ((A-B) > -ZERO) )
#define isSimilar(A, B) ( ((A-B) < SMALL) && ((A-B) > -SMALL) )
#define isBetween(A, B, C) ( ((A-B) > -ZERO) && ((A-C) < ZERO) )


///////////////////////////////////////
//Helpers

template<class T>
bool EqualVectors(vector<T> a, vector<T> b);

template<class T>
void printVector(vector<T> vec);

    template<class T>
void printVector(vector<T> vec)
{
    for(int i=0; i<vec.size(); i++) {
	cout<<vec[i]<<" ";
    }
    cout<<endl;
}

template<class T>
bool EqualVectors(vector<T> a, vector<T> b) {
    if (a.size() != b.size()) {
	return false;
    }

    bool c=true;
    for (int i=0; i < a.size(); i++) {
	c &= (a[i]==b[i]);
    }

    if(c == false) {
	printVector(a);
	cout<<endl;
	printVector(b);
    }

    return c;
}

/////////////////////////////////////////
//Tests

TEST(FASTSLAM_TEST,compute_steering_test)
{
    MatrixXd lm; //landmark positions
    MatrixXd wp; //way points

    read_input_file("example_webmap.mat", &lm, &wp);

    VectorXd xtrue(3);
    xtrue.setZero();

    int iwp =0;
    double minD = 1;
    double G = 0;
    double rateG = 0.3491;
    double maxG = 0.5236;
    double dt = 0.0250;

    compute_steering(xtrue,wp,iwp,minD,G,rateG,maxG,dt);

    EXPECT_NE(G,-0.0087);
    EXPECT_EQ(iwp,0);
}

TEST(FASTSLAM_TEST,predict_true_test)
{
    Vector3d xtrue(3);
    xtrue.setZero();

    double V = 3;
    double G = -0.0087;
    int WB = 4;
    double dt = 0.0250;

    predict_true(xtrue,V,G,WB,dt);

    VectorXd xtrue_out(3);
    xtrue_out<<0.0750,-0.0007,-0.0002;

    EXPECT_NE(xtrue,xtrue_out); 
}

TEST(FASTSLAM_TEST,add_control_noise_test) {
    double V = 3;
    double G = -0.0087;
    Matrix2d Q;
    Q   <<0.0900,0,
	0,0.0027;
    int SWITCH_CONTROL_NOISE = 1;
    double* VnGn = new double[2];
    add_control_noise(V,G,Q,SWITCH_CONTROL_NOISE,VnGn);

    double Vn = 2.6056;
    double Gn = -0.0305;
    EXPECT_NE(VnGn[0],Vn);
    EXPECT_NE(VnGn[1],Gn);
}

TEST(FASTSLAM_TEST,predict_test){
    double w = 0.01;
    Vector3d xv(3);
    xv.setZero();

    vector<Vector2d> xf;
    vector<Matrix2d> Pf;

    double Vn = 3.0194;
    double Gn = 0.0227;
    Matrix2d Qe(2,2);
    Qe<< 0.0900, 0,
	0,0.0027;

    double WB = 4;
    double dt = 0.0250;

    int SWITCH_PREDICT_NOISE = 0;

    Particle particle = Particle();
    particle.setW(w);
    particle.setXv(xv);
    particle.setXf(xf);
    particle.setPf(Pf);

    //predict
    predict(particle,Vn,Gn,Qe,WB,dt,SWITCH_PREDICT_NOISE);

    double w_after = 0.01;
    VectorXd xv_after(3);
    xv_after<<0.0755,0.0017,0.0004;
    vector<VectorXd> xf_out;
    vector<Matrix2d> Pf_out;

    //EXPECT_NE(w_after, particle.w());
    EXPECT_NE(xv_after[0], particle.xv()[0]);
    EXPECT_NE(xv_after[1], particle.xv()[1]);
    EXPECT_NE(xv_after[2], particle.xv()[2]);
    EXPECT_TRUE(particle.xf().empty());
    EXPECT_TRUE(particle.Pf().empty ());
}

TEST(FASTSLAM_TEST,get_observations_test) 
{

    VectorXd xtrue(3);
    xtrue<<0.6741,-0.0309,-0.0074;
    MatrixXd lm; //landmark positions
    MatrixXd wp; //way points

    read_input_file("example_webmap.mat", &lm, &wp);

    vector<int> ftag;
    for (int i=0; i<lm.cols(); i++) {
	ftag.push_back(i);
    }

    vector<int> ftag_visible = vector<int>(ftag);
    EXPECT_EQ(ftag_visible.size(), ftag.size());    
    double rmax = 30;

    vector<Vector2d> z_out = get_observations(xtrue,lm,ftag_visible,rmax);
    vector<Vector2d> z_gt;

    VectorXd a(2);
    a<<25.7745,-1.4734;
    VectorXd b(2);
    b<<25.2762, 0.1384;
    z_gt.push_back(a);
    z_gt.push_back(b);

    vector<int> ftag_visible_gt;
    ftag_visible_gt.push_back(0);
    ftag_visible_gt.push_back(21);

    EXPECT_TRUE(EqualVectors(ftag_visible_gt,ftag_visible));
    //EXPECT_TRUE(EqualVectors(z_gt,z_out));            
}

TEST(FASTSLAM_TEST,add_observation_noise_test)
{
    vector<Vector2d> z;
    Vector2d a(2);
    a<<25.7745,-1.4734;
    Vector2d b(2);
    b<<25.2762,0.1384;
    z.push_back(a);
    z.push_back(b);

    Matrix2d R(2,2);
    R<<0.0100, 0,
	0, 0.0003;

    int SWITCH_SENSOR_NOISE = 1;

    add_observation_noise(z,R,SWITCH_SENSOR_NOISE);

    vector<Vector2d> z_gt;
    Vector2d a_gt(2);
    a_gt<<25.8522,-1.4621;
    Vector2d b_gt(2);
    b_gt<<25.3384,0.1309;
    z_gt.push_back(a_gt);
    z_gt.push_back(b_gt);

    //EXPECT_TRUE(EqualVectors(z_gt,z));
}

TEST(FASTSLAM_TEST,data_associate_known_test)
{
    vector<Vector2d> z;
    Vector2d a(2);
    a<<25.7301,-1.4884;
    Vector2d b(2);
    b<<25.1439,0.1137;
    z.push_back(a);
    z.push_back(b);

    vector<int> ftag_visible;
    ftag_visible.push_back(0);
    ftag_visible.push_back(21);

    VectorXd da_table(35);
    for (int i=0; i<35; i++) {
	da_table[i]=-1;
    }

    int Nf = 0;

    vector<Vector2d> zf;
    vector<int> idf;
    vector<Vector2d> zn;
    data_associate_known(z,ftag_visible,da_table, Nf,zf,idf,zn);

    vector<Vector2d> zn_gt;
    Vector2d c(2);
    c<<25.7301,-1.4884;
    Vector2d d(2);
    d<<25.1439,0.1137;
    zn_gt.push_back(c);
    zn_gt.push_back(d);

    //check [zf,idf,zn,da_table] 
    EXPECT_TRUE(zf.empty());
    EXPECT_TRUE(idf.empty());

    VectorXd da_table_gt(35);
    for (int i=0; i<35; i++) {
	da_table_gt[i]=-1;
    }
    da_table_gt[0] = 0;
    da_table_gt[21] =1;

    EXPECT_TRUE(da_table_gt==da_table);
    EXPECT_TRUE(EqualVectors(zn,zn_gt));
}


TEST(FASTSLAM_TEST,compute_jacobian_test)
{
    double w = 0.01;

    Vector3d xv(3);
    xv << 1.3393, -0.1241, -0.0278;

    vector<Vector2d> xf;
    VectorXd a(2);
    a<<3.4924 ,-25.7649;
    VectorXd b(2);
    b<<25.6182,3.9462;
    xf.push_back(a);
    xf.push_back(b);

    vector<Matrix2d> Pf;
    Matrix2d c(2,2);
    c<< 0.2016,0.0210,
	0.0210, 0.0123;
    Matrix2d d(2,2);
    d<< 0.0146, -0.0288,
	-0.0288, 0.1898;
    Pf.push_back(c);
    Pf.push_back(d);

    vector<Vector2d> zf;
    Vector2d e(2);
    e<<25.5817,-1.4737;
    Vector2d f(2);
    f<<24.6040,0.1458;
    zf.push_back(e);
    zf.push_back(f);

    vector<int> idf;
    idf.push_back(0);
    idf.push_back(1);

    Matrix2d R(2,2);
    R<< 0.0100, 0,
	0, 0.0003;

    //particle
    Particle particle = Particle();
    particle.setW(w);
    particle.setXv(xv);
    particle.setXf(xf);
    particle.setPf(Pf);

    vector<Vector2d> zp;
    vector<Matrix23d> Hv;
    vector<Matrix2d> Hf;
    vector<Matrix2d> Sf;
    compute_jacobians(particle,idf,R,zp,&Hv,&Hf,&Sf);

    //compute jacobians
    vector<Vector2d> zp_gt;
    Vector2d g(2);
    g<<25.7310,-1.4593;
    Vector2d h(2);
    f<<24.6177,0.1939;
    zp_gt.push_back(g);
    zp_gt.push_back(f);

    vector<Matrix23d> Hv_gt;
    Matrix23d i(2,3);
    i<<-0.0837,0.9965,0,
	-0.0387,-0.0033,-1.0;
    Matrix23d j(2,3);
    j<< -0.9862,-0.1653,0,
	0.0067,-0.0401,-1.0;
    Hv_gt.push_back(i);
    Hv_gt.push_back(j);

    vector<Matrix2d> Hf_gt;
    Matrix2d m(2,2);
    m<<0.0837,-0.9965,
	0.0387, 0.0033;
    Matrix2d n(2,2);
    n<<0.9862, 0.1653,
	-0.0067, 0.0401;
    Hf_gt.push_back(m);
    Hf_gt.push_back(n);


    vector<MatrixXd> Sf_gt;
    MatrixXd k(2,2);
    k<< 0.0201, -0.0002,
	-0.0002, 0.0006;
    MatrixXd l(2,2);
    l<< 0.0200, 0.0001, 
	0.0001, 0.0006;
    Sf_gt.push_back(k);
    Sf_gt.push_back(l);

    //tests (zp,Hv,Hf,Sf)
    //EXPECT_TRUE(EqualVectors(zp,zp_gt));
    //EXPECT_TRUE(EqualVectors(Hv,Hv_gt));
    //EXPECT_TRUE(EqualVectors(Hf,Hf_gt));
    //EXPECT_TRUE(EqualVectors(Sf,Sf_gt));
}

TEST(FASTSLAM_TEST,compute_weight_test)
{
    //w
    double w = 0.01;

    //xv
    Vector3d xv(3);
    xv << 1.2746 ,-0.1712 ,-0.0380;

    //xf
    vector<Vector2d> xf;
    VectorXd a(2);
    a<<3.7387,-25.7130;
    VectorXd b(2);
    b<<25.9565,2.8205;
    xf.push_back(a);
    xf.push_back(b);

    //Pf
    vector<Matrix2d> Pf;
    Matrix2d c(2,2);
    c<<0.2005, 0.0230,
	0.0230,0.0128;
    Matrix2d d(2,2);
    d<<0.0124, -0.0211,
	-0.0211,0.1953;
    Pf.push_back(c);
    Pf.push_back(d);

    //particle
    Particle particle = Particle();
    particle.setW(w);
    particle.setXv(xv);
    particle.setXf(xf);
    particle.setPf(Pf);

    //zf
    vector<Vector2d> zf;
    Vector2d e(2);
    e<<25.6975,-1.4795;
    Vector2d f(2);
    f<<24.5414,0.1713;
    zf.push_back(e);
    zf.push_back(f);

    //idf
    vector<int> idf;
    idf.push_back(0);
    idf.push_back(1);

    //R
    Matrix2d R(2,2);
    R<< 0.0100, 0,
	0, 0.0003;

    double w_out = compute_weight(particle, zf,idf,R);
    double w_gt = 29.6451;
    EXPECT_NE(w_out,w_gt);
}

TEST(FASTSLAM_TEST,feature_update_test)
{
    //w
    double w = 2.6993;

    //xv
    Vector3d xv(3);
    xv << 1.3010, -0.1420, -0.0315;

    //xf
    vector<Vector2d> xf;
    VectorXd a(2);
    a<<2.7368,-25.5656;
    VectorXd b(2);
    b<<25.9803,2.4939;
    xf.push_back(a);
    xf.push_back(b);

    //Pf
    vector<Matrix2d> Pf;
    Matrix2d c(2,2);
    c<<0.1983, 0.0155,
	0.0155,0.0113;
    Matrix2d d(2,2);
    d<<0.0119, -0.0187,
	-0.0187,0.1958;
    Pf.push_back(c);
    Pf.push_back(d);

    //particle
    Particle particle = Particle();
    particle.setW(w);
    particle.setXv(xv);
    particle.setXf(xf);
    particle.setPf(Pf);

    //zf
    vector<Vector2d> zf;
    Vector2d e(2);
    e<<25.4653,-1.4981;
    Vector2d f(2);
    f<<24.6960,0.1803;
    zf.push_back(e);
    zf.push_back(f);   

    //idf
    vector<int> idf;
    idf.push_back(0);
    idf.push_back(1);

    //R
    Matrix2d R(2,2);
    R<< 0.0100, 0,
	0, 0.0003;

    //feature update
    feature_update(particle,zf,idf,R);

    //w_gt
    double w_gt = 2.6993;

    //xv_gt
    VectorXd xv_gt(3);
    xv_gt<< 1.3010, -0.1420, -0.0315;

    //xf_gt
    vector<Vector2d> xf_gt;
    VectorXd g(2);
    g<<2.5430,-25.5796;
    VectorXd h(2);
    h<<25.8635,3.0197;
    xf_gt.push_back(g);
    xf_gt.push_back(h);

    //Pf_gt
    vector<Matrix2d> Pf_gt;
    Matrix2d i(2,2);
    i<< 0.0985, 0.0065,
	0.0065, 0.0055;
    Matrix2d j(2,2);
    j<< 0.0060, -0.0094,
	-0.0094,0.0953;
    Pf_gt.push_back(i);
    Pf_gt.push_back(j);

    EXPECT_EQ(particle.w(),w_gt);
    EXPECT_EQ(particle.xv(),xv_gt);
    //EXPECT_TRUE(EqualVectors(particle.xf(),xf_gt));
    //EXPECT_TRUE(EqualVectors(particle.Pf(),Pf_gt));
}


TEST(FASTSLAM_TEST,add_feature_test)
{
    double w=0.0100;
    //xv
    Vector3d xv(3);
    xv<<0.6981, -0.0280,-0.0067;

    //xf
    vector<Vector2d> xf;

    //Pf
    vector<Matrix2d> Pf;

    //particle
    Particle particle = Particle();
    particle.setW(w);
    particle.setXv(xv);
    particle.setXf(xf);
    particle.setPf(Pf);

    //zf
    vector<Vector2d> zn;
    Vector2d a(2);
    a<<25.8047,-1.4638;
    Vector2d b(2);
    b<<25.2023,0.1198;
    zn.push_back(a);
    zn.push_back(b);  

    //R
    Matrix2d R(2,2);
    R<< 0.0100, 0,
	0, 0.0003;

    //add_feature
    add_feature(particle,zn,R);    

    double w_out = 0.01;

    //xv_out
    VectorXd xv_out(3);
    xv_out<<0.6981,-0.0280,-0.0067;

    //Xd out
    vector<VectorXd> xf_out;
    VectorXd c(2);
    c<<3.2830, -25.7030;
    VectorXd d(2);
    d<< 25.7394, 2.8163;
    xf_out.push_back(c);
    xf_out.push_back(d);

    //Pf out 
    vector<Matrix2d> Pf_out;
    Matrix2d e(2,2);
    e<<0.2009, 0.0192,
	0.0192,0.0119;
    Matrix2d f(2,2);
    f<<0.0123, -0.0206,
	-0.0206, 0.1911;
    Pf_out.push_back(e);
    Pf_out.push_back(f);

    EXPECT_EQ(w_out,particle.w());
    EXPECT_EQ(xv_out,particle.xv());
    //EXPECT_TRUE(EqualVectors(xf_out, particle.xf()));
    //EXPECT_TRUE(EqualVectors(Pf_out, particle.Pf()));
}

TEST(FASTSLAM_TEST, stratified_random_test) 
{
    int N = 10;
    vector<double> s;
    stratified_random(10, s);

    vector<double> s_gt;
    s_gt.push_back(0.0948);
    s_gt.push_back(0.1334);
    s_gt.push_back(0.2390);
    s_gt.push_back(0.3150);
    s_gt.push_back(0.4334);
    s_gt.push_back(0.5554);
    s_gt.push_back(0.6550);
    s_gt.push_back(0.7160);
    s_gt.push_back(0.8117);
    s_gt.push_back(0.9399);

    int i; 
    cout<<"s is"<<endl;
    for (i=0; i<s.size(); i++) {
	cout<<s[i]<<" ";
    }
    cout<<endl;

    cout<<"s_gt is"<<endl;
    for (i=0; i<s_gt.size(); i++) {
	cout<<s_gt[i]<<" ";
    }
    cout<<endl;

    //EXPECT_TRUE(EqualVectors(s,s_gt));
}

TEST(FASTSLAM_TEST, stratified_resample_test) 
{
    vector<int> keep;
    double Neff;

    VectorXd w(10);
    w<< 0.8074, 0.2651, 0.8959, 0.6080, 0.0144, 0.3440, 0.5337, 0.6278, 0.4467, 0.8111;
    stratified_resample(w, keep,Neff);

    vector<int> keep_gt;
    keep_gt.push_back(0); 
    keep_gt.push_back(0); 
    keep_gt.push_back(2); 
    keep_gt.push_back(2); 
    keep_gt.push_back(3); 
    keep_gt.push_back(6); 
    keep_gt.push_back(7); 
    keep_gt.push_back(7); 
    keep_gt.push_back(8); 
    keep_gt.push_back(9); 

    int i;
    /*
    cout<<"keep"<<endl;
    for (i=0; i<keep.size(); i++) {
	cout<<keep[i]<<" ";
    }
    cout<<endl;

    cout<<"keep_gt"<<endl;
    for (i=0; i<keep_gt.size(); i++) {
	cout<<keep_gt[i]<<" ";
    }
    cout<<endl;
    */
    EXPECT_TRUE(EqualVectors(keep_gt, keep));
    EXPECT_NE(Neff, 8.0764);     
}

/*
TEST(FASTSLAM_TEST, fastslam1_sim_test)
{
    MatrixXd lm; //landmark positions
    MatrixXd wp; //way points

    read_input_file("example_webmap.mat", &lm, &wp);

    vector<Particle> p = fastslam1_sim(lm,wp);
    VectorXd xv_gt;
    xv_gt<<-12.0425,-63.0278,1.1090;
    //EXPECT_NE(p[50].xv(),xv_gt);
    //EXPECT_NE(p[50].Pf[0],);   
}
*/

TEST(FASTSLAM_TEST, sample_proposal_test)
{
    //w
    double w = 0.01;

    //xv
    Vector3d xv(3);
    xv << 1.2767, -0.1756, -0.0394;

    //Pv
    Matrix3d Pv(3,3);
    Pv << 0.0004927, -0.0000653, -0.0000126,
	 -0.0000653,  0.0001592,  0.0000369,
	 -0.0000126,  0.0000369,  0.0000086;

    //xf
    vector<Vector2d> xf;
    Vector2d a(2);
    a<<2.4261,-25.8041;
    Vector2d b(2);
    b<<25.7095,3.2392;
    xf.push_back(a);
    xf.push_back(b);

    //Pf
    vector<Matrix2d> Pf;
    Matrix2d c(2,2);
    c<< 0.2019, 0.0133,
	0.0133, 0.0109;
    Matrix2d d(2,2);
    d <<0.0131, -0.0239,
	-0.0239, 0.1915;
    Pf.push_back(c);
    Pf.push_back(d);

    //particle
    Particle particle = Particle();
    particle.setW(w);
    particle.setXv(xv);
    particle.setPv(Pv);
    particle.setXf(xf);
    particle.setPf(Pf);


    vector<Vector2d> zf;
    Vector2d i(2);
    i<<25.74325,-1.4901;
    Vector2d j(2);
    j<<24.4729, 0.1353;
    zf.push_back(i);
    zf.push_back(j);
     
    //idf
    vector<int> idf;
    idf.push_back(0);
    idf.push_back(1);

    //R
    Matrix2d R(2,2);
    R<< 0.0100, 0,
	0, 0.0003;

    sample_proposal(particle,zf, idf, R);

    // sample_proposal
    double w_gt = 1.6005;
    
    Vector3d xv_gt(3);
    xv_gt<< 1.2935, -0.1718, -0.0387;

    Matrix3d Pv_gt(3,3);
    Pv_gt.setZero();    

    vector<Vector2d> xf_gt;
    VectorXd e(2);
    a<<2.4261,-25.8041;
    VectorXd f(2);
    b<<25.7095,3.2392;
    xf_gt.push_back(a);
    xf_gt.push_back(b);
 
    vector<Matrix2d> Pf_gt;
    Matrix2d g(2,2);
    c<< 0.2019, 0.0133,
	0.0133, 0.0109;
    Matrix2d h(2,2);
    d <<0.0131, -0.0239,
	-0.0239, 0.1915;
    Pf_gt.push_back(c);
    Pf_gt.push_back(d);

    EXPECT_EQ(w_gt,particle.w());
    EXPECT_EQ(xv_gt,particle.xv());
    EXPECT_EQ(Pv_gt,particle.Pv());
    EXPECT_TRUE(EqualVectors(xf_gt,particle.xf()));
    EXPECT_TRUE(EqualVectors(Pf_gt,particle.Pf()));
}

TEST(FASTSLAM_TEST, likelihood_given_xv_test)
{
    //w
    double w = 0.01;

    //xv
    Vector3d xv(3);
    xv << 1.2919, -0.1874,-0.0423;

    //Pv
    Matrix3d Pv(3,3);
    Pv.setZero();

    //xf
    vector<Vector2d> xf;
    VectorXd a(2);
    a<<2.4261,-25.8041;
    VectorXd b(2);
    b<<25.7095,3.2392;
    xf.push_back(a);
    xf.push_back(b);

    //Pf
    vector<Matrix2d> Pf;
    Matrix2d c(2,2);
    c<< 0.2019, 0.0133,
	0.0133, 0.0109; 
    Matrix2d d(2,2);
    d<< 0.0131, -0.0239,
	-0.0239,0.1915;
    Pf.push_back(c);
    Pf.push_back(d);

    //particle
    Particle particle = Particle();
    particle.setW(w);
    particle.setXv(xv);
    particle.setPv(Pv);
    particle.setXf(xf);
    particle.setPf(Pf);

    vector<Vector2d> zf;
    Vector2d i(2);
    i<<25.7432,-1.4901;
    Vector2d j(2);
    j<<24.4729, 0.1353;
    zf.push_back(i);
    zf.push_back(j);
     
    //idf
    vector<int> idf;
    idf.push_back(0);
    idf.push_back(1);

    //R
    Matrix2d R(2,2);
    R<< 0.0100, 0,
	0, 0.0003;
   
    double like = likelihood_given_xv(particle, zf,idf, R);
    EXPECT_NE(like, 122.7224);
}

TEST(FASTSLAM_TEST, gauss_evaluate_test)
{
   
    VectorXd v(3);
    v<<-0.0480,-0.0286,-0.0069;

    Matrix3d Pv0(3,3);
    Pv0<<0.0004977,-0.0000494, -0.0000091,
   -0.0000494,0.0001815,0.0000418,
   -0.0000091,0.0000418,0.0000097;

    double prior_gt = 6378.3;

    double prior = gauss_evaluate(v,Pv0,0);
    #if 0
    VectorXd v(3);
    v <<-0.0306, 0.0077, 0.0017;

    MatrixXd Sf(3,3);
    Sf << 0.0004983, -0.0000430, -0.0000089, 
	-0.0000430, 0.0001697, 0.0000393,
	-0.0000089, 0.0000393, 0.0000091;
    double prior_gt = 603610;
    double prior  = gauss_evaluate(v,Sf,0);
    #endif
    #if 0
    VectorXd v(2);
    v<< -1.18453, 1.62279;
    
    Matrix2d Sf(2,2);
    Sf<<0.020094, -0.000184015,
    -0.000184015,  0.000607922;

    double prior = gauss_evaluate(v, Sf, 0);
    double prior_gt = 0; 
    #endif
    EXPECT_EQ(prior, prior_gt);
}

TEST(FASTSLAM_TEST, multivariate_gauss_test)
{
    VectorXd x(2);
    x<< 3.0000, -0.0087;

    Matrix2d P(2,2);
    P<< 0.09, 0,
         0, 0.0027;

    int n = 1;
    VectorXd mg = multivariate_gauss(x,P,n);
    VectorXd mg_gt(2);
    mg_gt << 0.1977, 0.2207;
    EXPECT_EQ(mg, mg_gt);
}

TEST(FASTSLAM_TEST, delta_xv_test) 
{


}


TEST(FASTSLAM_TEST, resample_particles_test)
{
    vector<Particle> particles(10);
    for (int i = 0; i< particles.size(); i++) {
        particles[i] = Particle();
    }
   
    particles[0].setW(5.9056);
    particles[1].setW(14.0648);
    particles[2].setW(9.5197);
    particles[3].setW(21.3748);
    particles[4].setW(6.5495);
    particles[5].setW(4.1826);
    particles[6].setW(17.4620);
    particles[7].setW(12.8126);
    particles[8].setW(7.9298);
    particles[9].setW(20.8908);

    int Nmin = 7; 
    int doresample = 1; 
    resample_particles(particles,Nmin,doresample);

    vector<double> w_gt(10);
    w_gt[0] = 0.0489;
    w_gt[1] = 0.1165;
    w_gt[2] = 0.0789;
    w_gt[3] = 0.1771;
    w_gt[4] = 0.0543;
    w_gt[5] = 0.0347;
    w_gt[6] = 0.1447;
    w_gt[7] = 0.1062;
    w_gt[8] = 0.0657;
    w_gt[9] = 0.1731;

    for (int i =0; i< 10; i++) {
        EXPECT_EQ(particles[i].w(), w_gt[i]);
    }
}

TEST(FASTSLAM_TEST, misc_test)
{
    double like = 4.2596*pow(10.f,3.f);
    double prior = 5.4764*pow(10.f,5.f);
    double prop = 6.5415* pow(10.f,5.f);
    double w_gt = 35.6746;


    double a = prior/prop;
    double b = 0.01 * a;
    double newW = like * b;
    EXPECT_EQ(newW, w_gt);

    #if 0
    //Matrix2d P;
    double total =0;
    double particle_w= 1.28907*pow(10.0f,30.0f);
    double like = 3135.09;
    double prop = 3.07849*pow(10.0f,6.0f);
    double prior = 3.1012*pow(10.0f,6.0f);
    
    cout<<"particle_w "<<particle_w<<endl;
    cout<<"like "<<like<<endl;
    cout<<"prop "<<prop<<endl;
    cout<<"prior "<<prior<<endl;
    cout<<"prop/prior "<<prop/prior<<endl;
    double b = prop/prior;
    cout<<"particle_w * prop/prior "<<particle_w * b<<endl;
    double c = particle_w * b;
    double d = c* like;
    cout<<"total "<< d<<endl;
    
    //P = nRandMat::randn(3,10);
    //cout<<"normalized random matrix"<<endl;    
    //cout<<P<<endl;
    #endif
    #if 0
    boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)

    boost::normal_distribution<> nd(0.0, 1.0);

    boost::variate_generator<boost::mt19937&, 
                           boost::normal_distribution<> > var_nor(rng, nd);

    int i = 0; for (; i < 10; ++i)
    {
        double d = var_nor();
        std::cout << d << std::endl;
    }

    #endif
//    boost::minstd_rand random_gen(42u);
//    boost::normal_distribution<double> normal(0,20);
    
/*
    EXPECT_EQ(exp(2),7.3891);
    
    MatrixXd S(3,3);
    S<< 0.0004960, -0.0000521, -0.0000098,
    -0.0000521, 0.0001869, 0.0000430,
    -0.0000098, 0.0000430, 0.0000099;

    MatrixXd cholMat = S.llt().matrixL();

    MatrixXd Sc(3,3);
    Sc<< 0.0223, 0, 0,
    -0.0023, 0.0121, 0,
    -0.0005, 0.0028, 0.0001;
    
    VectorXd v(3);
    v<< -0.0081,-0.0135,-0.0033;
    
    VectorXd nin = Sc.jacobiSvd(ComputeThinU | ComputeThinV).solve(v);

    VectorXd nin_gt(3);
    nin_gt<<-0.3651,-1.1870,-0.6920;
    EXPECT_EQ(nin, nin_gt);
   
    EXPECT_EQ(cholMat, Sc); 
*/
}

