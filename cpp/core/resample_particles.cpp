#include <math.h>
#include "resample_particles.h"
#include "configfile.h"
#include "stratified_resample.h"
#include <iostream>
#include <assert.h>

using namespace std;

void resample_particles(vector<Particle> &particles, int Nmin, int doresample) 
{
    int N = particles.size();
    VectorXd w(N);
    w.setZero();

    for (int i=0; i<N; i++) {
        w(i) = particles[i].w();
    }

    double ws = w.sum();
    if (ws == 0) {
        for (int i = 0; i < N; i++) {
            w(i) = 1./N;
        }
        ws = w.sum();
    }
    /* assert(ws != 0); */

    for (int i=0; i<N; i++) {
        w(i) = w(i)/ws;
    }

    for (int i=0; i<N; i++) {
        particles[i].setW(particles[i].w()/ws);
    }

    double Neff=0;
    vector<int> keep;
    stratified_resample(w,keep,Neff);

    vector<Particle> old_particles = vector<Particle>(particles);
    particles.resize(keep.size());	
    //do I need to clear this? 


    if ((Neff < Nmin) && (doresample == 1)) {
        for(int i=0; i< keep.size(); i++) {
            particles[i] = old_particles[keep[i]]; 	
        }	
        for (int i=0; i<N; i++) {
            double new_w = 1.0f/(double)N;
            assert(isfinite(new_w));
            assert(N == 100);
            particles[i].setW(new_w);
        }
    }		
}
