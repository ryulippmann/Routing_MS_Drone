// Routing_MS_Drone.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include "class_def.h"
#include "cluster.h"

using namespace std;

int ReefPt::count = 0;
int Pt::count = 0;
int MS::count = 0;
int Tender::count = 0;
int Cluster::count = 0;
int ClusterSoln::count = 0;

int main()
{
    cout << "Hello World!\n";
	vector<ReefPt> reefPts;
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
		reefPts.push_back(ReefPt(rnd_num, 0));
			cout << reefPts[i].ID << "\t(" << reefPts[i].x << ", " << reefPts[i].y << ")" << endl;
	}

	int numClust =3;
	Pt depot = Pt(0,0);
	int numTenders = 2;
	int tenderCap = 2;

	Problem inst(reefPts, numClust, depot, numTenders, tenderCap);		// create instance of problem
	
	ClusterSoln init_solution(kMeansConstrained(
		inst.getReefPointers(), inst.getnumClusters(), 1000, false));
	for (int i = 0; i < init_solution.clusters.size(); i++) {
		cout << "Cluster " << i << endl;
		for (int j = 0; j < init_solution.clusters[i]->reefs.size(); j++) {
			cout << init_solution.clusters[i]->reefs[j]->ID << "\t(" << init_solution.clusters[i]->reefs[j]->x << ", " << init_solution.clusters[i]->reefs[j]->y << ")" << endl;
		}
		cout << "Centroid: " << init_solution.clusters[i]->getCentroid().ID << "\t(" << init_solution.clusters[i]->getCentroid().x << ", " << init_solution.clusters[i]->getCentroid().y << ")" << endl;
	}
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
