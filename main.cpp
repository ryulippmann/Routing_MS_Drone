// Routing_MS_Drone.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
using namespace std;

#include <iostream>

#include "class_prob.h"
//#include "MSSoln.h"
#include "calcs.h"
#include "class_soln.h"
#include "cluster.h"
#include "mothership.h"
#include "tenders.h"
//#include "swaps.h"


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
    cout << "Hello World!\n";
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
	for (int i = 0; i < 12; i++) {
		int rnd_num_x = distribution(gen);						// Generate a random number
		int rnd_num_y = distribution(gen);						// Generate a random number
		reefPts.push_back(Pt(rnd_num_x, rnd_num_y));
			cout << reefPts[i].ID << "\t(" << reefPts[i].x << ", " << reefPts[i].y << ")" << endl;
	}

	int numClust =3;
	Pt depot = Pt(0,0);
	int numTenders = 2;
	int tenderCap = 2;

	Problem inst(reefPts, numClust, depot, numTenders, tenderCap);		// create instance of problem
	///////////////// Problem Initialised /////////////////
	
	////////////   Cluster Soln Construction - function out   ////////////
	//\\//\\//\\//\\// Create clusters \\//\\//\\//\\//
	vector<ClusterSoln*> clusters = kMeansConstrained(
		inst, 1000, false);
	
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
	msSoln.clustSolns = clusterCentroidNearestNeighbour(inst, clusters);
	greedyMSCluster(inst, msSoln);						// Improve using Gd 2-Opt: update clustSoln.clustOrder
	// ^^ unconfirmed if this is working correctly ^^

	vector<vector<double>> dMatrix_launchpt = setLaunchPts(inst, msSoln);			// Set MS Launchpts: msSoln.launchPts
	
	double msDist = msSoln.getDist();
	vector<Pt*> ms_route = msSoln.getRoute();
	//\\//\\//\\//\\//   MS Soln Initialised   //\\//\\//\\//\\//
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	//\\//\\//\\//\\// TenderSoln Construction //\\//\\//\\//\\//
	vector<TenderSoln*> tenderSolns;
		// for each cluster
	for (int c = 0; c < clusters.size(); c++)	{										// for each cluster
		pair<Pt*, Pt*> launchPts = make_pair(msSoln.launchPts[c], msSoln.launchPts[c + 1]);		// launchPts for cluster
		vector<vector<double>> clusterMatrix = msSoln.clustSolns[c]->getdMatrix(c, launchPts);	// distance matrix for cluster
		Pt centroid = clusters[c]->getCentroid();										// centroid for cluster	
		printf("\nCluster %d\tcentroid: (%.2f, %.2f)\n", centroid.ID, centroid.x, centroid.y);
		// print distance matrix
		for (int i = 0; i < clusterMatrix.size(); i++) { for (int j = 0; j < clusterMatrix[i].size(); j++) { printf("%.2f\t", clusterMatrix[i][j]); } printf("\n"); } printf("\n");
		//\\//\\//\\//\\// TenderSoln Initialised //\\//\\//\\//\\//
		// Tendersoln Nearest Neighbour
		TenderSoln clustTendersoln (clusters[c], droneWithinClusterNearestNeighbour(&msSoln, c), 
			make_pair(msSoln.launchPts[c], msSoln.launchPts[c + 1]) );		//vector<vector<Pt*>> cluster_routes = droneWithinClusterNearestNeighbour(&msSoln, c);		
		
		// Tendersoln Greedy 2-Opt update
		clustTendersoln.routes = greedyTenderCluster(&clustTendersoln, msSoln.clustSolns[c]->getdMatrix(c, make_pair(msSoln.launchPts[c], msSoln.launchPts[c + 1])));

		tenderSolns.push_back(&clustTendersoln);
		// print routes
		for (int i = 0; i < clustTendersoln.routes.size(); i++) {
			printf("Route %d:\n", i);
			for (int j = 0; j < clustTendersoln.routes[i].size(); j++) {
				printf("%d\t(%.2f, %.2f)\n", clustTendersoln.routes[i][j]->ID, clustTendersoln.routes[i][j]->x, clustTendersoln.routes[i][j]->y);
			}// for each node in route
		}// for each route
	}// for each cluster
	FullSoln gd(&msSoln, tenderSolns);
	////////////////////////////////

	// Tendersoln Swaps

	// Tendersoln Swaps: Out
	//string filename = SwapFunction(gd, 0);

	// Tendersoln Swaps: In


	////////////////////////////////


	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}

