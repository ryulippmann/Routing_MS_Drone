using namespace std;

#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include <random>

#include "class_prob.h"
int numClust = 10;//3;//
Pt depot = Pt(0, 0);		// depot must be first point initialised! ID = 0
int numTenders = 2;
int tenderCap = 5;//2;//
int no_pts = 100;
//if (numClust * numTenders * tenderCap != no_pts) throw invalid_argument("numClust * numTenders * tenderCap != no_pts");
// create instance of problem
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
//for (int i = 0; i < 12; i++) {
//	ReefPt rp(i+1,0);
//	reefPts.push_back(rp);
//	cout << rp.getID() << "\t(" << rp.getXY().first << ", " << rp.getXY().second << ")" << endl;
//}

random_device rd;
mt19937 gen(0);
uniform_int_distribution<int> distribution(1, 10);		// Define the distribution for integers between 1 and 10 (inclusive)

int main()
{
	//cout << "Hello World!\n";
	
	////////////   Cluster Soln Construction - function out   ////////////
	//\\//\\//\\//\\// Create clusters \\//\\//\\//\\//
	vector<ClusterSoln*> clusters = kMeansConstrained(/*1000*/100000, false);
	//\\//\\//\\//\\// Clusters created \\//\\//\\//\\//
	// PRINT clusters //
	for (int i = 0; i < clusters.size(); i++) {
		printf("\tCluster: %d\n", i);					// Print clusters
		for (int j = 0; j < clusters[i]->reefs.size(); j++) {	// for reefs in cluster
			printf("%d\t(%.2f, %.2f)\n", clusters[i]->reefs[j]->ID, clusters[i]->reefs[j]->x, clusters[i]->reefs[j]->y);
		} // print ID (x,y) for each reef in cluster
		printf("Centroid:\t\t%d\t(%.2f, %.2f)\n", clusters[i]->getCentroid().ID, clusters[i]->getCentroid().x, clusters[i]->getCentroid().y);
	} // for each cluster
	//\\//\\//\\//\\// Cluster Soln Initialised //\\//\\//\\//\\//
	
	//\\//\\//\\//\\//   MS Soln Construction   //\\//\\//\\//\\//
	MSSoln msSoln(clusters);
	//\\//\\//\\// clustOrder for MS route solution \\//\\//\\//
	msSoln.clusters = clusterCentroidNearestNeighbour(clusters);		// clusters ordered by NN
	// print MSSoln total route dist
	setLaunchPts(msSoln);
	printf("\n%.2f\n", msSoln.getDist());

	greedyMSCluster(msSoln);						// Improve using Gd 2-Opt: update clustSoln.clustOrder
	// print MSSoln total route dist
	setLaunchPts(msSoln);
	printf("\n%.2f\n", msSoln.getDist());
	
	double msDist = msSoln.getDist();
	vector<Pt*> ms_launch_route = msSoln.getRoute();
	//\\//\\//\\//\\//   MS Soln Initialised   //\\//\\//\\//\\//
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	//\\//\\//\\//\\// TenderSoln Construction //\\//\\//\\//\\//
	vector<TenderSoln> tenderSolns;
	for (int c = 0; c < clusters.size(); c++) {		// for each cluster
		pair<Pt*, Pt*> launchPts = make_pair(msSoln.launchPts[c], msSoln.launchPts[c + 1]);		// launchPts for cluster
		vector<vector<double>> clusterMatrix = msSoln.clusters[c]->getdMatrix(launchPts);	// distance matrix for cluster
		Pt centroid = msSoln.clusters[c]->getCentroid();										// centroid for cluster	
		printf("\nCluster %d\tcentroid: (%.2f, %.2f)\n", msSoln.clusters[c]->ID, centroid.x, centroid.y);
		// print distance matrix
		for (int i = 0; i < clusterMatrix.size(); i++) { for (int j = 0; j < clusterMatrix[i].size(); j++) { printf("%.2f\t", clusterMatrix[i][j]); } printf("\n"); } printf("\n");
		//\\//\\//\\//\\// TenderSoln Initialised //\\//\\//\\//\\//
		// Tendersoln Nearest Neighbour
		vector<vector<Pt*>> tenderRoutes = TenderWithinClusterNearestNeighbour(msSoln, c);
		TenderSoln tenderSoln (*msSoln.clusters[c],
			tenderRoutes,//TenderWithinClusterNearestNeighbour(msSoln, c),
			launchPts);		
		// Tendersoln Greedy 2-Opt update
		tenderSoln.routes = greedyTenderCluster(tenderSoln, clusterMatrix);

		// FIXED - deep copy operator: printf("\nERROR HERE - main L97 - ADDING tenderSoln to tenderSolns...\nIs this line necessary/doing anything?\n");
		tenderSolns.emplace_back(tenderSoln);
		// print routes
		for (int i = 0; i < tenderSoln.routes.size(); i++) { printf("Route %d:\n", i);
			for (int j = 0; j < tenderSoln.routes[i].size(); j++) {
				printf("%d\t(%.2f, %.2f)\n", tenderSoln.routes[i][j]->ID, tenderSoln.routes[i][j]->x, tenderSoln.routes[i][j]->y);
			}// for each node in route
		}// for each route
	}// for each cluster
	vector<TenderSoln*> ptr_tenderSolns;
	for (const auto& soln : tenderSolns) {
		ptr_tenderSolns.push_back(new TenderSoln(soln)); // Assuming TenderSoln has a copy constructor
	}
	FullSoln gd(msSoln, ptr_tenderSolns);
	////////////////////////////////
	printf("\nGd Dist: \t%.2f", gd.getTotalDist());
	// Tendersoln Swaps

	// Tendersoln Swaps: In
	//bool in_out = 1;
	///*string filename*/ 
	//FullSoln best_in = SwapFunction(gd, in_out);
	//printf("\nGd Dist: \t%.2f", gd.getTotalDist());
	//printf("\nIn_Swap distance:\t%.2f\n", best_in.getTotalDist());

	// Tendersoln Swaps: Out
	bool in_out = 0;
	FullSoln best_out = SwapFunction(gd/*best_in*/, in_out);
	printf("\nGd Dist: \t%.2f", gd.getTotalDist());
	//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
	printf("\nOut_Swap distance:\t%.2f\n", best_out.getTotalDist());


	////////////////////////////////


	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}

