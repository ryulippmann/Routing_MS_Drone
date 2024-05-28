using namespace std;
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include "class_prob.h"

bool csv_print = 1;
bool print_detail = 0;

int Pt::count = 0;

///////////////// Problem Initialised /////////////////

#include "calcs.h"
#include "class_soln.h"
#include "cluster.h"
#include "prints.h"
#include "mothership.h"
#include "drones.h"
#include "swaps.h"
#include "sensitivity.h"

int MS::count = 0;
int Drone::count = 0;
int ClusterSoln::count = 0;
int MSSoln::count = 0;
int DroneSoln::count = 0;
int FullSoln::count = 0;

int main()
{
	//vector <pair < pair<int, int>, pair<double, FullSoln> >> results  =
	//	VaryNum_droneXclust();
	const Problem inst =
		//CreateInst(100, 5, 5, 4, make_pair(2,1), Pt(0, 0), pow(10, 0));
		CreateInst(48, 4, 4, 3, make_pair(2, 1), Pt(0, 0), 0); //  Base case instance!
	//	no_pts, noClust, noDrones, dCap, make_pair(w_ms, w_d));


	if (checkParameters(inst) == false) return 0;		// check if parameters are valid
	printSetup(inst);		// print problem setup
	vector<FullSoln> fullSolns_best;

	bool flag_full_run = 1;
	bool flag_sens_weights = 0;
	bool flag_sens_clusters = 0;
	bool flag_sens_drones = 0;
	int sens_iterations = 3;
	if (flag_full_run+flag_sens_weights+flag_sens_clusters+flag_sens_drones != 1) {
		cout << "Select exactly one sensitivity analysis option!" << endl;
		return 0;
	}

	string sens_run;
	vector<vector<FullSoln>> fullSolns;
	if (flag_full_run) {
		int iter = 3;
		for (int i = 0; i < iter; i++) {
			fullSolns.push_back(
				FullRun(i, inst, inst.time + "_FullRuns")
			);
		}
		
		for (int i = 0; i < fullSolns.size(); i++) { fullSolns_best.push_back((fullSolns[i].back())); }
		printOpts(inst, fullSolns_best, "_FullRuns");
	}
	else if (flag_sens_weights) {
		vector <pair < pair<double, double>, pair<double, FullSoln> >> weight_results =
			VaryWeights(
				inst, sens_run,
				make_pair(1,10),
				10, sens_iterations);
		
		for (int i = 0; i < weight_results.size(); i++) { fullSolns_best.push_back((weight_results[i].second.second)); }
		printOpts(inst, fullSolns_best, sens_run);
	} 
	else if (flag_sens_clusters) {
		vector <pair < pair<int, int>, pair<double, FullSoln> >> noclust_results =
			VaryClusters(
				inst, sens_run, sens_iterations);
		
		for (int i = 0; i < noclust_results.size(); i++) { fullSolns_best.push_back((noclust_results[i].second.second)); }
		printOpts(inst, fullSolns_best, sens_run);
	}
	else if (flag_sens_drones) {
		vector <pair < pair<int, int>, pair<double, FullSoln> >> dCap_results =
			VaryDrones(
				inst, sens_run, sens_iterations);
		
		for (int i = 0; i < dCap_results.size(); i++) { fullSolns_best.push_back((dCap_results[i].second.second)); }
		printOpts(inst, fullSolns_best, sens_run);
	}

	//\\//\\//\\//\\//\\       FIN        //\\//\\//\\//\\//\\
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//	
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}
