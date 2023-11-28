// Routing_MS_Drone.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include "class_prob.h"
//#include "MSSoln.h"
#include "calcs.h"
#include "class_soln.h"
#include "cluster.h"
#include "mothership.h"
#include "tenders.h"

using namespace std;

int Pt::count = 0;
int MS::count = 0;
int Tender::count = 0;

int Cluster::count = 0;
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
		int rnd_num = distribution(gen);						// Generate a random number
		reefPts.push_back(Pt(rnd_num, 0));
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
	vector<Cluster*> clusters = kMeansConstrained(
		inst.getReefPointers(), inst.ms.cap, 1000, false);
	
	ClusterSoln clustSoln(clusters);
	for (int i = 0; i < clustSoln.clusters.size(); i++) {
		printf("\tCluster: %d\n", i);					// Print clusters
		for (int j = 0; j < clustSoln.clusters[i]->reefs.size(); j++) {	// for reefs in cluster
			printf("%d\t(%.2f, %.2f)\n", clustSoln.clusters[i]->reefs[j]->ID, clustSoln.clusters[i]->reefs[j]->x, clustSoln.clusters[i]->reefs[j]->y);
		}
		printf("Centroid:\t\t%d\t(%.2f, %.2f)\n", clustSoln.clusters[i]->getCentroid().ID, clustSoln.clusters[i]->getCentroid().x, clustSoln.clusters[i]->getCentroid().y);
	}
	clustSoln.centroidMatrix = calc_centMatrix(clustSoln.clusters, inst.ms.depot);	// Create centroid matrix
	//\\//\\//\\//\\// Cluster Soln Initialised //\\//\\//\\//\\//
	//\\//\\//\\//\\//   MS Soln Construction   //\\//\\//\\//\\//
	MSSoln msSoln(& inst, & clustSoln);
	//\\//\\//\\// clustOrder for MS route solution \\//\\//\\//
	msSoln.clustSoln->clusters = clusterCentroidNearestNeighbour(clustSoln);
	greedyCluster(msSoln);						// Improve using Gd 2-Opt: update clustSoln.clustOrder
	// ^^ unconfirmed if this is working correctly ^^

	setLaunchPts(msSoln/*, clustSoln*/);			// Set MS Launchpts: msSoln.launchPts

	vector<vector<double>> dm = msSoln.ordered_dMatrix();
	for (int i = 0; i < dm.size(); i++) {
		for (int j = 0; j < dm[i].size(); j++) {
			printf("%.2f\t", dm[i][j]);
		}
		printf("\n");
	}
	double msDist = msSoln.getDist();
	vector<Pt*> route = msSoln.getRoute();
	//\\//\\//\\//\\//   MS Soln Initialised   //\\//\\//\\//\\//
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	//\\//\\//\\//\\// TenderSoln Construction //\\//\\//\\//\\//

	//	// Function to find Pt by ID
	//auto findPtByID = [&pts](int targetID) -> const Pt* {
	//	auto it = std::find_if(pts.begin(), pts.end(), [targetID](const Pt& pt) {
	//		return pt.ID == targetID;
	//		});

	//	return (it != pts.end()) ? &(*it) : nullptr;
	//	};

	vector<TenderSoln> tenderSolns;
	for (int c = 0; c < clustSoln.clusters.size(); c++)	{
		// , &clustSoln.clusters[c]->reefs
		//TenderSoln _tenderSoln_(clustSoln.clusters[c]);	// Create TenderSoln for each cluster
		pair<Pt*, Pt*> launchPts = make_pair(msSoln.launchPts[c], msSoln.launchPts[c + 1]);
		vector<vector<double>> dmm = msSoln.clustSoln->clusters[c]->getdMatrix(c, launchPts);
		Pt centroid = clustSoln.clusters[c]->getCentroid();
		printf("\nCluster %d\tcentroid: (%.2f, %.2f)\n", centroid.ID, centroid.x, centroid.y);
		for (int i = 0; i < dmm.size(); i++) {
			for (int j = 0; j < dmm[i].size(); j++) {
				printf("%.2f\t", dmm[i][j]);
			}
			printf("\n");
		}
		printf("\n");
		//\\//\\//\\//\\// TenderSoln Initialised //\\//\\//\\//\\//
		// Tendersoln Nearest Neighbour
		vector<vector<Pt*>> cluster_routes = droneWithinClusterNearestNeighbour(/*&inst, clustSoln.clusters[c], */&msSoln, c);
		
		TenderSoln clustTendersoln (clustSoln.clusters[c], cluster_routes, make_pair(msSoln.launchPts[c], msSoln.launchPts[c + 1]));
		
		// Tendersoln 2-Opt
		////////////////////////////////
		cluster_routes = greedyCluster(&clustTendersoln , &msSoln, /*clustSoln.clusters[c], cluster_routes,*/ c);
			//droneWithinCluster2Opt(&msSoln, c);
		////////////////////////////////
		
		// Tendersoln Swaps
	

		// Tendersoln Swaps: In
	

		// Tendersoln Swaps: Out


	}//(&clustSoln.clusters, &msSoln);

	

	//\\//\\//\\//\\// TenderSoln Initialised //\\//\\//\\//\\//

	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	printf("\n\n");
	//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
