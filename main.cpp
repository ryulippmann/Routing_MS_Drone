// Routing_MS_Drone.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
using namespace std;

#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

#include "class_prob.h"
//#include "MSSoln.h"
#include "calcs.h"
#include "class_soln.h"
#include "cluster.h"
#include "mothership.h"
#include "tenders.h"
#include "swaps.h"


int Pt::count = 0;
int MS::count = 0;
int Tender::count = 0;

//int Cluster::count = 0;
int ClusterSoln::count = 0;
int MSSoln::count = 0;
int TenderSoln::count = 0;
int FullSoln::count = 0;

int main()
{
	std::cout << "Hello World!\n";
	vector<Pt> reefPts;
 //   for (int i = 0; i < 12; i++) {
	//	ReefPt rp(i+1,0);
	//	reefPts.push_back(rp);
	//	cout << rp.getID() << "\t(" << rp.getXY().first << ", " << rp.getXY().second << ")" << endl;
	//}

	#include <random>
	random_device rd;
	mt19937 gen(0);
	uniform_int_distribution<int> distribution(1, 10);		// Define the distribution for integers between 1 and 10 (inclusive)

	int numClust = 10;//3;
	// depot must be first point initialised! ID = 0
	Pt depot = Pt(0,0);
	int numTenders = 2;
	int tenderCap = 5;//2;

	//for (int i = 0; i < 12; i++) {
	//	int rnd_num_x = distribution(gen);						// Generate a random number
	//	int rnd_num_y = distribution(gen);						// Generate a random number
	//	reefPts.push_back(Pt(rnd_num_x, rnd_num_y));
	//		cout << reefPts[i].ID << "\t(" << reefPts[i].x << ", " << reefPts[i].y << ")" << endl;
	//}
	//Problem inst(reefPts, numClust, depot, numTenders, tenderCap);		// create instance of problem
	Problem inst(initReefs(), numClust, depot, numTenders, tenderCap);
	///////////////// Problem Initialised /////////////////
	
	////////////   Cluster Soln Construction - function out   ////////////
	//\\//\\//\\//\\// Create clusters \\//\\//\\//\\//
	vector<ClusterSoln*> clusters = kMeansConstrained(
		inst, /*1000*/100000, false);
	
	//ClusterSoln clustSoln(clusters);
	// print clusters
	for (int i = 0; i < clusters.size(); i++) {
		printf("\tCluster: %d\n", i);					// Print clusters
		for (int j = 0; j < clusters[i]->reefs.size(); j++) {	// for reefs in cluster
			printf("%d\t(%.2f, %.2f)\n", clusters[i]->reefs[j]->ID, clusters[i]->reefs[j]->x, clusters[i]->reefs[j]->y);
		}
		printf("Centroid:\t\t%d\t(%.2f, %.2f)\n", clusters[i]->getCentroid().ID, clusters[i]->getCentroid().x, clusters[i]->getCentroid().y);
	} 
	// this is a class method of ClusterSoln
	//clusters.centroidMatrix = calc_centMatrix(clusters, inst.ms.depot);	// Create centroid matrix
	
	//\\//\\//\\//\\// Cluster Soln Initialised //\\//\\//\\//\\//
	
	//\\//\\//\\//\\//   MS Soln Construction   //\\//\\//\\//\\//
	MSSoln msSoln(inst, clusters);
	//\\//\\//\\// clustOrder for MS route solution \\//\\//\\//
	msSoln.clusters = clusterCentroidNearestNeighbour(inst, clusters);		// clusters ordered by NN
	// print MSSoln total route dist
	setLaunchPts(inst, msSoln);
	std::cout << "\n" << msSoln.getDist() << "\n";

	greedyMSCluster(inst, msSoln);						// Improve using Gd 2-Opt: update clustSoln.clustOrder
	// print MSSoln total route dist
	setLaunchPts(inst, msSoln);
	std::cout << "\n" << msSoln.getDist() << "\n";

	//vector<vector<double>> dMatrix_launchpt = setLaunchPts(inst, msSoln);			// Set MS Launchpts: msSoln.launchPts
	
	double msDist = msSoln.getDist();
	vector<Pt*> ms_launch_route = msSoln.getRoute();
	//\\//\\//\\//\\//   MS Soln Initialised   //\\//\\//\\//\\//
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	//\\//\\//\\//\\// TenderSoln Construction //\\//\\//\\//\\//
	vector<TenderSoln> tenderSolns;
		// for each cluster
	for (int c = 0; c < clusters.size(); c++)	{										// for each cluster
		pair<Pt*, Pt*> launchPts = make_pair(msSoln.launchPts[c], msSoln.launchPts[c + 1]);		// launchPts for cluster
		vector<vector<double>> clusterMatrix = msSoln.clusters[c]->getdMatrix(launchPts);	// distance matrix for cluster
		Pt centroid = msSoln.clusters[c]->getCentroid();										// centroid for cluster	
		printf("\nCluster %d\tcentroid: (%.2f, %.2f)\n", msSoln.clusters[c]->ID, centroid.x, centroid.y);
		// print distance matrix
		for (int i = 0; i < clusterMatrix.size(); i++) { for (int j = 0; j < clusterMatrix[i].size(); j++) { printf("%.2f\t", clusterMatrix[i][j]); } printf("\n"); } printf("\n");
		//\\//\\//\\//\\// TenderSoln Initialised //\\//\\//\\//\\//
		// Tendersoln Nearest Neighbour
		TenderSoln tenderSoln (msSoln.clusters[c], 
			droneWithinClusterNearestNeighbour(&msSoln, c),
			launchPts);		//vector<vector<Pt*>> cluster_routes = droneWithinClusterNearestNeighbour(&msSoln, c);		
		
		// Tendersoln Greedy 2-Opt update
		tenderSoln.routes = greedyTenderCluster(&tenderSoln, clusterMatrix);

		printf("\nERROR HERE - ADDING tenderSoln to tenderSolns...\n");
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
	FullSoln gd(&msSoln, ptr_tenderSolns);
	////////////////////////////////

	// Tendersoln Swaps

	// Tendersoln Swaps: In
	bool in_out = 1;
	string filename = SwapFunction(gd, in_out);

	// Tendersoln Swaps: Out


	////////////////////////////////


	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}

