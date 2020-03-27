#include "stratified_random.h"
#include <assert.h>
#include <iostream>

using namespace std;

void stratified_random(int N, vector<double> &di)
{ 
    double k = 1.0/(double)N;
   
    //deterministic intervals
    double temp = k/2;
    di.push_back(temp);
    while (temp < 1-k/2) {
        temp = temp+k;
        di.push_back(temp);
    }
    /* 
    cout<<"di"<<endl;
    for (int i=0; i< di.size(); i++) {
	cout<<di[i]<<" "<<endl;
    }
 
    cout<<"N "<<N<<endl;
    cout<<"di size "<<di.size()<<endl; 
    //assert(di.size() == N); 
    */
    //dither within interval
    vector<double>::iterator diter; 
    for (diter = di.begin(); diter != di.end(); diter++) {
        *diter = (*diter) + unifRand() * k - (k/2);
    }
}

double unifRand()
{
    return rand() / double(RAND_MAX);
}
