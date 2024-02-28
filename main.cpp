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

int main()
{
	if (numClust * numTenders * tenderCap != no_pts) 
		throw invalid_argument("numClust * numTenders * tenderCap != no_pts");

	////////////   ClusterSoln Construction   ////////////
	//\\//\\//\\//\\// Create clusters \\//\\//\\//\\//
	vector<ClusterSoln*> clusters = kMeansConstrained(1000/*0*/, false);
	printClusters(clusters);		// PRINT clusters //
	//\\//\\//\\//\\  ClusterSoln Initialised \\//\\//\\//\\//
	
	//\\//\\//\\//\\//  MsSoln Construction //\\//\\//\\//\\//
	MSSoln msSoln(clusters);
	//\\//\\//\\// clustOrder for MS route solution \\//\\//\\//
	double msDist;// = msSoln.getDist();
	msDist = clusterCentroidNearestNeighbour(msSoln);		// clusters ordered by NN
	msDist = greedyMSCluster(msSoln);						// Improve using Gd 2-Opt: update clustSoln.clustOrder
	//vector<Pt*> ms_launch_route = msSoln.getRoute();

	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	//\\//\\//\\//\\/   MsSoln Initialised   /\\//\\//\\//\\//
	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//\\//\\//\\//\\  TenderSoln Construction  \\//\\//\\//\\/
	vector<TenderSoln> tenderSolns = initTenderSoln(clusters, msSoln);
	vector<TenderSoln*> ptr_tenderSolns;
	for (const auto& soln : tenderSolns) { ptr_tenderSolns.push_back(new TenderSoln(soln)); } // Assuming TenderSoln has a copy constructor
	//\\//\\//\\//\   TenderSoln Initialised   \//\\//\\//\\//
	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//\\//\\//\\//\\/  FullSoln Construction  /\\//\\//\\//\\/
	FullSoln gd(msSoln, ptr_tenderSolns);
	printf("\nGd Dist: \t%.2f", gd.getTotalDist());
	////////////////////////////////
	// Tendersoln Swaps
	FullSoln best = gd;	

	//// Tendersoln Swaps: Out
	bool out = 0;
	FullSoln best_out = SwapShell(gd, out);
	printf("\nGd Dist: \t%.2f", gd.getTotalDist());		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
	printf("\nOut_Swap distance:\t%.2f\n", best_out.getTotalDist());
	best = best_out;

	// Tendersoln Swaps: In
	bool in = 1;	/*string filename*/ 
	FullSoln best_in = SwapShell(best, in);
	printf("\nGd Dist: \t%.2f", gd.getTotalDist());
	printf("\nIn_Swap distance:\t%.2f\n", best_in.getTotalDist());
	best = best_in;

	////////////////////////////////

	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}

