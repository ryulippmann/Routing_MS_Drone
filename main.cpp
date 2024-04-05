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

// create GLOBAL instance of problem
const Problem inst =
//CreateInst(100, 5, 5, 4, make_pair(2,1), Pt(0, 0), pow(10, 0));
	CreateInst(48, 4,  4, 3, make_pair(1, 1), Pt(0, 0), 1); //  Base case instance!
	//	no_pts, noClust, noDrones, dCap, make_pair(w_ms, w_d));

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
	if (checkParameters(inst) == false) return 0;		// check if parameters are valid
	printSetup(inst);		// print problem setup

	vector<vector<FullSoln>> fullSolns;
	int iter = 10;
	for (int i = 0; i < iter; i++) {
		fullSolns.push_back(FullRun(i));
	}
	
	////\\//\\//\\//\\  ClusterSoln Construction  //\\//\\//\\//
	//vector<ClusterSoln*> clusters = kMeansConstrained(inst.kMeansIters);
	//if (print_detail) printClusters(clusters);		// PRINT clusters //
	//if (csv_print) createFolder();
	//if (csv_print) csvPrintClusters(clusters, "clusters_init");		// CSV PRINT clusters //
	////\\//\\//\\//\\  ClusterSoln Initialised \\//\\//\\//\\//
	////\\    \\//    //\\    \\//    //\\    \\//    //\\    \\	
	////\\//\\//\\//\\//  MsSoln Construction //\\//\\//\\//\\//
	//MSSoln msSoln(clusters);				// No launchPts initialised yet
	//vector<pair<double,MSSoln>> msSolns = initMsSoln(clusters, msSoln, print_detail/*, csv_print*/);
	////\\//\\//\\//\\/   MsSoln Initialised   /\\//\\//\\//\\//
	////\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	////\\//\\//\\//\\  DroneSoln Construction  \\//\\//\\//\\/
	//vector<DroneSoln> droneSolns = initDroneSoln(msSoln, print_detail);
	//vector<DroneSoln*> ptr_droneSolns;
	//for (const auto& soln : droneSolns) { ptr_droneSolns.push_back(new DroneSoln(soln)); } // Assuming DroneSoln has a copy constructor
	////\\//\\//\\//\   DroneSoln Initialised   \//\\//\\//\\//		
	////\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	////\\//\\//\\//\\/  FullSoln Construction  /\\//\\//\\//\\/
	//FullSoln full_init(msSoln, ptr_droneSolns);
	//printf("Full Soln Dist:\t%.2f", full_init.getTotalDist(print_detail));
	//if (csv_print) csvPrints(full_init, "INIT");
	//////////////////////////////////
	//vector<FullSoln> fullSolns;
	//fullSolns.push_back(full_init);
	//vector<double> best_dist{ fullSolns.back().getTotalDist()};			// initialise best_dist as vector with best solution distance	
	////\\//\\//\\//\\   FullSoln Initialised   \\//\\//\\//\\//		
	////\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//fullSolns.push_back(
	//	BaseSwapRun(fullSolns.back(), best_dist)
	//);

	//\\//\\//\\//\\//\\       FIN        //\\//\\//\\//\\//\\
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//	
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}
	//// Dronesoln Swaps
	//bool in_out = 0; //0;
	//while (best_dist.size() < 3 || best_dist.back() != best_dist.at(best_dist.size() - 3))
	//{
	//	FullSoln best_new = SwapShell(fullSolns.back(), in_out);
	//	printf("\nPrev Dist: \t\t%.2f", best_dist.back());		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
	//	best_dist.push_back(best_new.getTotalDist());
	//	printf("\n%d\tNEW_Swap distance:\t%.2f\n", in_out, best_dist.back());
	//	// vv fullSolns is not creating new fullSoln objects, but rather just pointing to the same object
	//	fullSolns.push_back(best_new);
	//	// csv print if solution updated
	//	if (csv_print && best_dist.back() != best_dist.at(best_dist.size() - 2)) {
	//		csvPrints(best_new, in_out);
	//	}
	//	in_out = !in_out;				// switch in_out flag
	//	//best = best_new;
	//}
	////////////////////////////////

