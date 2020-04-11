#include <iostream>
#include <math.h>
#include <vector>
#include <iomanip>

#include "fastslam1_sim.h"
#include "core/add_control_noise.h"
#include "core/get_observations.h"
#include "core/add_observation_noise.h"
#include "core/TransformToGlobal.h"
#include "core/line_plot_conversion.h"
#include "core/data_associate_known.h"
#include "core/feature_update.h"
#include "core/resample_particles.h"
#include "core/add_feature.h"
#include "core/particle_to_json.h"
#include "compute_weight.h"
#include "predict.h"
#include "core/estimate_json.h"
#include "core/ground_truth_json.h"
using namespace config;
using namespace std;

#include <iostream>

#define WRITE_TRACE 1

int counter=0;
vector<Particle> fastslam1_sim(MatrixXd lm, MatrixXd wp) 
{
    if (SWITCH_PREDICT_NOISE) {
	printf("Sampling from predict noise usually OFF for FastSLAM 2.0\n");	
    }

    //normally initialized configfile.h
    Q << pow(sigmaV,2), 0,
      0 , pow(sigmaG,2);

    R << sigmaR*sigmaR, 0, 
      0, sigmaB*sigmaB;

    double veh[2][3] = {{0,-WHEELBASE,-WHEELBASE},{0,-1,1}};

    //vector of particles (their count will change)
    vector<Particle> particles(NPARTICLES);
    for (int i=0; i<particles.size(); i++){
	particles[i] = Particle();
    }

    //initialize particle weights as uniform
    double uniformw = 1.0/(double)NPARTICLES;    
    for (unsigned int p = 0; p < NPARTICLES; p++) {
	particles[p].setW(uniformw);
    }

    Vector3d xtrue(3);
    xtrue.setZero();

    double dt = DT_CONTROLS; //change in time btw predicts
    double dtsum = 0; //change in time since last observation
    double dtsum_total = 0;

    vector<int> ftag; //identifier for each landmark
    for (int i=0; i< lm.cols(); i++) {
	ftag.push_back(i); 
    }

    //data ssociation table
    VectorXd da_table(lm.cols());
    for (int s=0; s<da_table.size(); s++) {
	da_table[s] = -1;
    }

    int iwp = 0; //index to first waypoint
    double G = 0; //initial steer angle
    MatrixXd plines; //will later change to list of points

    if (SWITCH_SEED_RANDOM !=0) {
	srand(SWITCH_SEED_RANDOM);
    } 		

    Matrix2d Qe = Matrix2d(Q);
    Matrix2d Re = Matrix2d(R);

    if (SWITCH_INFLATE_NOISE ==1) {
	Qe = 2*Q;
	Re = 2*R;
    }

    vector<int> ftag_visible;
	ftag_visible.resize(0);

    vector<Vector2d> z; //range and bearings of visible landmarks

	vector<int> idf;
	int cnt = 0;
	bool observe=true;
	int id=0;

    nlohmann::json particle_trace = {{"timesteps", nlohmann::json::array()}};

	nlohmann::json ground_truth = {{"timesteps", nlohmann::json::array()}};

	ground_truth.update(ground_truth_keypoints_json(wp, lm));
    //Main loop
	
    while (iwp !=-1) {
		cnt++;

		if (cnt==60000) {
			break;
		}
	
#if WRITE_TRACE
	//std::cerr << "adding another round of particles at waypoint " << iwp << std::endl;
	/*
	auto relevant_particles = std::vector<Particle>{};
	std::copy_if(particles.begin(), particles.end(),
		     std::back_inserter(relevant_particles),
		     [](auto p){return p.w() > 0.001;});

	auto particle_jsons = std::vector<nlohmann::json>(relevant_particles.size());
	std::transform(relevant_particles.begin(), relevant_particles.end(),
		       particle_jsons.begin(), particle_to_json);
	*/
	//std::cout<<std::fmod(dtsum_total+0.001,DT_OBSERVE)<<" dtsum: "<<dtsum_total<<std::endl;
	if (observe) {
		particle_trace["timesteps"] += estimate_step_json(particles, dtsum_total, ftag_visible, da_table, id);
		observe=false;
	}

	ground_truth["timesteps"]+= ground_truth_step_json(xtrue, dtsum_total, id, iwp, G);
	id++;


	/*
	auto max_particle_iter = std::max_element(particles.begin(), particles.end(), [](auto p_lhs, auto p_rhs) {return p_lhs.w() < p_rhs.w();});
	particle_trace["timesteps"] += {
	    {"timestamp", dtsum_total}, 
	    {"max_particle", particle_to_json(*max_particle_iter)},
	    {"all_particle_poses", particle_poses_to_json(particles)}
	*/


	/*
	particle_trace["timesteps"] += {
	    {"timestamp", dtsum}, 
	    {"particles", std::accumulate(particle_jsons.begin(), particle_jsons.end(),
					  nlohmann::json::array(), [](auto lhs, auto rhs) 
					  {lhs += rhs; return lhs;})}
					  */
	//};
#endif
	compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
	if (iwp ==-1 && NUMBER_LOOPS > 1) {
	    iwp = 0;
	    NUMBER_LOOPS = NUMBER_LOOPS-1;
	}
	predict_true(xtrue,V,G,WHEELBASE,dt);

	//add process noise
	double* VnGn = new double[2];        
	add_control_noise(V,G,Q,SWITCH_CONTROL_NOISE,VnGn);
	double Vn = VnGn[0];
	double Gn = VnGn[1];

	//Predict step	
	for (unsigned int i=0; i< NPARTICLES; i++) {
	    predict(particles[i],Vn,Gn,Qe,WHEELBASE,dt,SWITCH_PREDICT_NOISE);
	    /* if (SWITCH_HEADING_KNOWN) { */
		/* for (int j=0; j< particles[i].xf().size(); j++) { */
		    /* Vector2d xf_j = particles[i].xf()[j]; */
		    /* xf_j[2] = xtrue[2]; */
		    /* particles[i].setXfi(j,xf_j); */	
		/* } */           
	    /* } */
	}

	//Observe step
	dtsum = dtsum+dt;
	dtsum_total += dt;
	
	//std::cout<<dtsum<<" vs. "<<DT_OBSERVE<<std::endl;
	if (dtsum >= DT_OBSERVE) {

		observe=true;
	    dtsum=0;

	    //Compute true data, then add noise
	    ftag_visible = vector<int>(ftag); //modify the copy, not the ftag	

	    //z is the range and bearing of the observed landmark
	    z = get_observations(xtrue,lm,ftag_visible,MAX_RANGE);

		/*
		for (auto elem : z) {
			std::cout<<elem<<std::endl;
		}
		*/
	    add_observation_noise(z,R,SWITCH_SENSOR_NOISE);

	    if (!z.empty()){
		plines = make_laser_lines(z,xtrue);
	    }

	    //Compute (known) data associations
	    int Nf = particles[0].xf().size();
	    //vector<int> idf;
	    idf.clear();
		vector<Vector2d> zf;
	    vector<Vector2d> zn;            

	    bool testflag= false;
	    data_associate_known(z,ftag_visible,da_table,Nf,zf,idf,zn);
	    
	    //perform update

		
	    for (int i =0; i<NPARTICLES; i++) {
		if (!zf.empty()) { //observe map features
		    double w = compute_weight(particles[i],zf,idf,R);
		    w = particles[i].w()*w;
		    particles[i].setW(w);
		    feature_update(particles[i],zf,idf,R);
		}
		if (!zn.empty()) {
		    add_feature(particles[i], zn, R);
		}
	    }

		

		

	    resample_particles(particles,NEFFECTIVE,SWITCH_RESAMPLE);

	    if (VnGn) { 
		delete[] VnGn;
	    }
	}
    }
    cout<<"done with all functions and will return particles"<<endl<<flush;
#if WRITE_TRACE
    std::ofstream of(output_filename);
    //of << std::setw(4) << particle_trace;
    of << particle_trace;


	std::ofstream of_gt(ground_truth_filename);
    //of << std::setw(4) << particle_trace;
    of_gt << ground_truth;
#endif
    return particles;
}

//rb is measurements
//xv is robot pose
MatrixXd make_laser_lines(vector<Vector2d> rb, Vector3d xv) 
{
    if (rb.empty()) {
	return MatrixXd(0,0);
    }

    int len = rb.size();
    MatrixXd lnes(4,len);

    MatrixXd globalMat(2,rb.size());
    int j;
    for (j=0; j<globalMat.cols();j++) {
	globalMat(0,j) = rb[j][0]*cos(rb[j][1]); 
	globalMat(1,j) = rb[j][0]*sin(rb[j][1]);   	
    }

    TransformToGlobal(globalMat,xv);

    for (int c=0; c<lnes.cols();c++) {
	lnes(0,c) = xv(0);
	lnes(1,c) = xv(1);
	lnes(2,c) = globalMat(0,c);
	lnes(3,c) = globalMat(1,c);
    }

    return line_plot_conversion(lnes);
}
