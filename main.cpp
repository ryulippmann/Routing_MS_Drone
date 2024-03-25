using namespace std;
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include "class_prob.h"
Pt depot = Pt(0, 0);		// depot must be first point initialised! ID = 0

int no_pts = 48; //100;
// validity check later: (numClust * numTenders * tenderCap = no_pts)
int numClust = 4;//5;
int numTenders = 4;
int tenderCap = 3;

//int no_pts = 12;
//// validity check later: (numClust * numTenders * tenderCap = no_pts)
//int numClust = 3;
//int numTenders = 2;
//int tenderCap = 2;

double w_ms = 1;
double w_d = 1;

bool csv_print = 1;
bool print_detail = 0;

// create GLOBAL instance of problem
const Problem inst(initReefs(no_pts), numClust, depot, numTenders, tenderCap, make_pair(w_ms, w_d));
///////////////// Problem Initialised /////////////////

#include "calcs.h"
#include "class_soln.h"
#include "cluster.h"
#include "prints.h"
#include "mothership.h"
#include "tenders.h"
#include "swaps.h"

int Pt::count = 0;
int MS::count = 0;
int Tender::count = 0;
int ClusterSoln::count = 0;
int MSSoln::count = 0;
int TenderSoln::count = 0;
int FullSoln::count = 0;
//vector<Pt> reefPts;

int main()
{
	if (checkParameters() == false) return 0;		// check if parameters are valid
	printSetup(inst);		// print problem setup
	////////////   ClusterSoln Construction   ////////////
	//\\//\\//\\//\\// Create clusters \\//\\//\\//\\//
	int kMeansIters = pow(10,2);//1000
	vector<ClusterSoln*> clusters = kMeansConstrained(kMeansIters);
	if (print_detail) printClusters(clusters);		// PRINT clusters //
	if (csv_print) createFolder();
	if (csv_print) csvPrintClusters(clusters, "clusters_init", kMeansIters);		// CSV PRINT clusters //
	
	//\\//\\//\\//\\  ClusterSoln Initialised \\//\\//\\//\\//
	
	//\\//\\//\\//\\//  MsSoln Construction //\\//\\//\\//\\//
	MSSoln msSoln(clusters);				// No launchPts initialised yet
	vector<pair<double,MSSoln>> msSolns = initMsSoln(clusters, msSoln, print_detail/*, csv_print*/);
	//vector<Pt*> ms_launch_route = msSoln.getRoute();
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	//\\//\\//\\// clustOrder for MS route solution \\//\\//\\//
	// MSSoln msSoln
	// vector<pair<double, MSSoln>> msSolns
	//\\//\\//\\//\\/   MsSoln Initialised   /\\//\\//\\//\\//

	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//\\//\\//\\//\\  TenderSoln Construction  \\//\\//\\//\\/
	vector<TenderSoln> tenderSolns = initTenderSoln(msSoln, print_detail);
	vector<TenderSoln*> ptr_tenderSolns;
	for (const auto& soln : tenderSolns) { ptr_tenderSolns.push_back(new TenderSoln(soln)); } // Assuming TenderSoln has a copy constructor
	//\\//\\//\\//\   TenderSoln Initialised   \//\\//\\//\\//
	// vector<TenderSoln> tenderSolns
	// vector<TenderSoln*> ptr_tenderSolns
		
	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//\\//\\//\\//\\/  FullSoln Construction  /\\//\\//\\//\\/
	FullSoln full_init(msSoln, ptr_tenderSolns);
	printf("Full Soln Dist:\t%.2f", full_init.getTotalDist(print_detail));
	if (csv_print) csvPrints(full_init, "INIT");
	////////////////////////////////
	vector<FullSoln> fullSolns;
	fullSolns.push_back(full_init);
	vector<double> best_dist{ fullSolns.back().getTotalDist()};			// initialise best_dist as vector with best solution distance	
	////////////////////////////////
	// Randomly run IN/OUT Swaps
	FullSoln best = SwapRandomly(fullSolns.back(), print_detail, csv_print);
	printf("\nPrev Dist: \t\t%.2f", best_dist.back());		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
	best_dist.push_back(best.getTotalDist());
	printf("\n\t\tNEW_Swap distance:\t%.2f\n", best_dist.back());
	// vv fullSolns is not creating new fullSoln objects, but rather just pointing to the same object
	fullSolns.push_back(best);
	// csv print if solution updated
	if (csv_print && best_dist.back() != best_dist.at(best_dist.size() - 2)) {csvPrints(best, "FINAL"); }
	////////////////////////////////

	//// Tendersoln Swaps
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

	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}