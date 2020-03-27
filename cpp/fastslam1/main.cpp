#include "fastslam1_sim.h"
#include "core/read_input_file.h"
#include "core/particle.h"
#include "core/utilities.h"

using namespace Eigen;
using namespace std;

int main (int argc, char *argv[])
{
	MyTimer Timer = MyTimer();
	Timer.Start();
	MatrixXd lm; //landmark positions
	MatrixXd wp; //way points

	if (argc < 2)
		return -1;

	read_input_file(argv[1], &lm, &wp);
	vector<Particle> data = fastslam1_sim(lm,wp);
	for (int i=0; i<data.size(); i++) {
		cout<<"particle i="<<i<<endl;
		cout<<endl;
		cout<<"weight"<<endl;
		cout<<data[i].w()<<endl;
		cout<<endl;
		cout<<"xv (robot pose)"<<endl;
		cout<<data[i].xv()<<endl;
		cout<<endl;
		cout<<"Pv (controls)"<<endl;
		cout<<data[i].Pv()<<endl;
		cout<<endl;
		cout<<"xf (EFK means)"<<endl;
		for(int j=0; j<data[i].xf().size(); j++) {
			cout<<data[i].xf()[j]<<endl;
			cout<<endl;
		}
		cout<<endl;
		cout<<"Pf (covariance mat)"<<endl;
		for(int k=0; k<data[i].Pf().size(); k++) {
			cout<<data[i].Pf()[k]<<endl;
			cout<<endl;
		}
		cout<<endl;
		cout<<endl;
	}
	Timer.Stop();
	Timer.Print("fastslam 1.0 ");
}
