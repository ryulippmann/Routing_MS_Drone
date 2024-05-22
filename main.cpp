using namespace std;
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include "class_prob.h"

bool csv_print = 1;
bool print_detail = 0;

int Pt::count = 0;
//Pt depot = Pt(0, 0);		// depot must be first point initialised! ID = 0
//// validity check later: (noClust * noDrones * dCap = no_pts)
//int no_pts = 48; //100;
//int noClust = 4;//5;
//int noDrones = 4;//5;
//int dCap = 3;//4;
//
////int no_pts = 12;
////// validity check later: (noClust * noDrones * dCap = no_pts)
////int noClust = 3;
////int noDrones = 2;
////int dCap = 2;
//
//double w_ms = 2;
//double w_d = 1;
//int kMeansIters = pow(10,7);//1000

//// create GLOBAL instance of problem
//const Problem INST =
////CreateInst(100, 5, 5, 4, make_pair(2,1), Pt(0, 0), pow(10, 0));
//	CreateInst(48, 4,  4, 3, make_pair(2, 1), Pt(0, 0), 0); //  Base case instance!
//	//	no_pts, noClust, noDrones, dCap, make_pair(w_ms, w_d));

///////////////// Problem Initialised /////////////////

#include "calcs.h"
#include "class_soln.h"
#include "cluster.h"
#include "prints.h"
//#include "opt.h"
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

	vector<vector<FullSoln>> fullSolns;
	int iter = 10;
	for (int i = 0; i < iter; i++) {
		fullSolns.push_back(
			FullRun(i, inst)
		);
	}
	for (int i = 0; i < fullSolns.size(); i++) { fullSolns_best.push_back( (fullSolns[i].back()) ); }
	printOpts(inst, fullSolns_best);

	//vector <pair < pair<double, double>, pair<double, FullSoln> >> weight_results = 
	//	VaryWeights(
	//		inst, 
	//		make_pair(1, 10), 
	//		make_pair(0.1, 1),
	//		3);
	//for (int i = 0; i < weight_results.size(); i++) { fullSolns_best.push_back( (weight_results[i].second.second) ); }

	printOpts(inst, fullSolns_best);

	//\\//\\//\\//\\//\\       FIN        //\\//\\//\\//\\//\\
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//	
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}
