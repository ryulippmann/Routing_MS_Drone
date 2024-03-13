using namespace std;

#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

#include "class_prob.h"
Pt depot = Pt(0, 0);		// depot must be first point initialised! ID = 0

int no_pts = 100;
int numClust = 10;//3;//
							// validity check later: (numClust * numTenders * tenderCap = no_pts)
int numTenders = 2;
int tenderCap = 5;//2;//

// create GLOBAL instance of problem
const Problem inst(initReefs(no_pts), numClust, depot, numTenders, tenderCap);
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

vector<Pt> reefPts;

bool csv_print = 0;

int main()
{
	if (numClust * numTenders * tenderCap != no_pts) 
		throw invalid_argument("numClust * numTenders * tenderCap != no_pts");

	////////////   ClusterSoln Construction   ////////////
	//\\//\\//\\//\\// Create clusters \\//\\//\\//\\//
	int kMeansIters = 1000;
	vector<ClusterSoln*> clusters = kMeansConstrained(kMeansIters, false);
	printClusters(clusters);		// PRINT clusters //
	if (csv_print) csvPrintClusters(clusters, "clusters_init", kMeansIters);		// CSV PRINT clusters //
	// vector<ClusterSoln*> clusters
	//\\//\\//\\//\\  ClusterSoln Initialised \\//\\//\\//\\//
	
	//\\//\\//\\//\\//  MsSoln Construction //\\//\\//\\//\\//
	MSSoln msSoln(clusters);				// No launchPts initialised yet
	vector<pair<double,MSSoln>> msSolns = initMsSoln(clusters, msSoln, csv_print);
	//vector<Pt*> ms_launch_route = msSoln.getRoute();
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	//\\//\\//\\// clustOrder for MS route solution \\//\\//\\//
	// MSSoln msSoln
	// vector<pair<double, MSSoln>> msSolns;
	//\\//\\//\\//\\/   MsSoln Initialised   /\\//\\//\\//\\//

	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//\\//\\//\\//\\  TenderSoln Construction  \\//\\//\\//\\/
	vector<TenderSoln> tenderSolns = initTenderSoln(msSoln, true);
	vector<TenderSoln*> ptr_tenderSolns;
	for (const auto& soln : tenderSolns) { ptr_tenderSolns.push_back(new TenderSoln(soln)); } // Assuming TenderSoln has a copy constructor
	//\\//\\//\\//\   TenderSoln Initialised   \//\\//\\//\\//
	// vector<TenderSoln> tenderSolns
	// vector<TenderSoln*> ptr_tenderSolns

	
	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//\\//\\//\\//\\/  FullSoln Construction  /\\//\\//\\//\\/
	FullSoln gd(msSoln, ptr_tenderSolns);
	printf("\nGd Dist: \t%.2f", gd.getTotalDist());
	csvPrints(gd);

	////////////////////////////////
	vector<FullSoln> fullSolns;
	//FullSoln best = gd;	// initialise best as gd
	fullSolns.push_back(gd);
	vector<double> best_dist{ fullSolns.back().getTotalDist()};			// initialise best_dist as vector with best solution distance
	////////////////////////////////
	// Tendersoln Swaps

	////// Tendersoln Swaps: Out
	//bool in_out = 0;
	//FullSoln best_out = SwapShell(fullSolns.back(), in_out);
	//printf("\nGd Dist: \t\t%.2f", best_dist.back()/*at(0)*/);		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
	//best_dist.push_back(best_out.getTotalDist());
	//printf("\nOut_Swap distance:\t%.2f\n", best_dist.back());
	//fullSolns.push_back(best_out);
	////best = best_out;

	//// Tendersoln Swaps: In
	//in_out = 1;
	//FullSoln best_in = SwapShell(best, in_out);
	//printf("\nGd Dist: \t%.2f", gd.getTotalDist());
	//best_dist.push_back(best.getTotalDist());
	//printf("\nIn_Swap distance:\t%.2f\n", best_dist.back());
	//best = best_in;
	bool in_out = 1;
	while (best_dist.size() < 3 || best_dist.back() != best_dist.at(best_dist.size() - 2))
	{
		in_out = !in_out;				// switch in_out flag
		FullSoln best_new = SwapShell(fullSolns.back(), in_out);
		printf("\nPrev Dist: \t\t%.2f", best_dist.back());		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
		best_dist.push_back(best_new.getTotalDist());
		printf("\n%d\tNEW_Swap distance:\t%.2f\n", in_out, best_dist.back());
		// vv fullSolns is not creating new fullSoln objects, but rather just pointing to the same object
		fullSolns.push_back(best_new);
		csvPrints(best_new, in_out);
		//best = best_new;
	}
	////////////////////////////////



	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}