#include "stratified_resample.h"
#include "stratified_random.h"
#include <iostream>

using namespace std;

void stratified_resample(VectorXd w, vector<int> &keep, double &Neff)
{
    VectorXd wsqrd(w.size());
    double wsum = w.sum();    

    for (int i=0; i<w.size(); i++) {
        w(i) = w(i)/ wsum;
        wsqrd(i) = (double)pow(w(i),2);
    }
    
    Neff = 1.0f/(double)wsqrd.sum();

    int len = w.size();
    keep.resize(len);
    for (int i=0; i<len; i++) {
        keep[i] = -1;
    }

    vector<double> select;
    stratified_random(len,select); 
    /*
    select.push_back(0.0948);
    select.push_back(0.1334);
    select.push_back(0.239);
    select.push_back(0.315);
    select.push_back(0.4334);
    select.push_back(0.5554);
    select.push_back(0.655);
    select.push_back(0.716);
    select.push_back(0.8117); 
    select.push_back(0.9399); 
    */
    cumsum(w);    

    int ctr=0;
    for (int i=0; i<len; i++) {
        while ((ctr<len) && (select[ctr]<w(i))) {
            keep[ctr] = i;
            ctr++;
        }
    }
}


void cumsum(VectorXd &w) 
{
    VectorXd csumVec = VectorXd(w);

    for (int i=0; i< w.size(); i++) {
        double sum =0;
        for (int j=0; j<=i; j++) {
	    sum+=csumVec(j);
	}			
	w(i) = sum;
    }
}
