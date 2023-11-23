#pragma once
using namespace std;
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

//#include "MSSoln.h"
//class MSSoln;

//struct Pt {
//public:
//	Pt(double x, double y) : ID(count++), x(x), y(y) {}
//	Pt(pair<double, double> coords) : Pt(coords.first, coords.second) {}
//
//	//int getID() const { return ID; }
//	const int ID;
//	const double x;
//	const double y;
//	//pair<double, double> getXY() const { return make_pair(x, y); }
//
//	//ReefPt& operator=(const ReefPt& point) {	// Assignment operator
//	//	//pair<double, double> coords = point.getXY();
//	//	if (this != &point) { ReefPt(point.x, point.y); }
//	//	return *this;	// self-assignment check	
//	//}
//private:
//	static int count;
//};

struct Pt {
public:
	Pt(double x = 0.0, double y = 0.0) : ID(count++), x(x), y(y) {}	// Default constructor
	Pt(pair<double, double> coords) : ID(count++), x(coords.first), y(coords.second) {}

	const int ID;
	const double x;
	const double y;
	//pair<double, double> getXY() const { return make_pair(x, y); }
private:
	static int count;
};

struct MS {
public:
	MS() : ID(count++), cap(NULL) {}
	MS(Pt depot) : ID(count++), /*cap(cap), */depot(depot) {}
	MS(int cap) : ID(count++), cap(cap) {}
	MS(int cap, Pt depot) : ID(count++), cap(cap), depot(depot) {}

	const int ID;
	const int cap=NULL;
	const Pt depot;	
	//int getID() const { return ID; }
	//int getCap() const { return cap; }
	//Pt getDepot() const { return depot; }

private:
	static int count;
};

struct Tender {
public:
	Tender(int cap) : ID(count++), cap(cap)/*, route(route)*/ {}

	const int ID;
	const int cap;
	//Route_Tender* route;
	//Route_Tender* getRoute() { return route; }
private:
	static int count;
};

const struct Problem {
private:
	const int numClust;
	const Pt depot;

	const int tenderCap;
	const int numTenders;

	MS setMS(int msCap, Pt depot) {
		MS ms(msCap, depot);
		return ms;
	}

	vector<Tender> setTenders(int numTenders, int tenderCap) {
		vector<Tender> tenders;
		for (int i = 0; i < numTenders; ++i) { tenders.push_back(Tender(tenderCap)); }
		return tenders;
	}

public:
	Problem(vector<Pt> reefs, int numClust, Pt depot, int numTenders, int tenderCap) :
		reefs(reefs), numClust(numClust), depot(depot), numTenders(numTenders), tenderCap(tenderCap)
		//ms(MS(numClust)), msCap(msCap), tenderCap(tenderCap), numTenders(numTenders)
	{}
	//Problem(vector<ReefPt> reefs, int numClust, Pt depot, int numTenders, int tenderCap) :
	//	reefs(reefs), numClust(numClust), depot(depot), numTenders(numTenders), tenderCap(tenderCap),
	//	ms(MS())//, msCap(msCap), tenderCap(tenderCap), numTenders(numTenders)
	//{}
	vector<Pt*> getReefPointers() const { 
		vector<Pt*> reefPtrs;
		for (auto& reef : reefs) {
			reefPtrs.push_back(const_cast<Pt*>(&reef));
		}
		return reefPtrs; 
	}
	//int getnumClusters() const { return numClust; }
	//Pt getDepot() const { return depot; }
	//MS getMS() const { return ms; }
	//int getnumTenders() const { return numTenders; }
	//vector<Tender> getTenders() const { return tenders; }
	vector<vector<double>> getDMatrix(vector<Pt> reefs, Pt depot) {
		vector<vector<double>> dMatrix;
		vector<double> depotDists;
		depotDists.push_back(0.0);
		for (auto& reef : reefs) {
			double dist = sqrt(pow(depot.x - reef.x, 2) + pow(depot.y - reef.y, 2));
			depotDists.push_back(dist);
		}
		dMatrix.push_back(depotDists);
		int r = 0;
		for (auto& reef_a : reefs) {
			r++;
			vector<double> row;
			row.push_back(depotDists[r]);
			for (auto& reef_b : reefs) {
				double dist = sqrt(pow(reef_a.x - reef_b.x, 2) + pow(reef_a.y - reef_b.y, 2));
				row.push_back(dist);
			}
			dMatrix.push_back(row);
		}
		return dMatrix;
	}
	const vector<vector<double>> dMatrix = this->getDMatrix(reefs, depot);
	const vector<Pt> reefs;

	const MS ms = this->setMS(numClust, depot);

	const vector<Tender> tenders = this->setTenders(numTenders, tenderCap);
};

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

struct Cluster {
public:
	Cluster(vector<Pt*> reefs/*, Pt start, Pt end*/) : ID(count++), reefs(reefs)/*, start(start), end(end)*/ {}

	const int ID;
	const vector<Pt*> reefs;
	//Pt centroid = this->getCentroid(/*reefs*/);
	Pt getCentroid() {
		double x = 0.0;
		double y = 0.0;
		for (auto& reef : reefs) {
			x += reef->x;
			y += reef->y;
		}
		Pt centroid(x/reefs.size(), y/reefs.size());
		return centroid;
	}

	vector<vector<double>> getdMatrix(const int c, pair <Pt*, Pt*> launchpts) {	//mothership, cluster

		vector<double> launchDists;
		launchDists.push_back(0.0);
		for (auto& reef : reefs) {
			double dist = sqrt(pow(reef->x - launchpts.first->x, 2) + pow(reef->y - launchpts.first->y, 2));
			launchDists.push_back(dist);
		}
		launchDists.push_back(DBL_MAX);

		vector<vector<double>> dMatrix;
		dMatrix.push_back(launchDists);

		vector<double> retrieveDists;
		retrieveDists.push_back(0.0);
		for (auto& reef : reefs) {
			double dist = sqrt(pow(reef->x - launchpts.second->x, 2) + pow(reef->y - launchpts.second->y, 2));
			retrieveDists.push_back(dist);
		}
		retrieveDists.push_back(0.0);

		for (int r = 0; r < reefs.size(); r++) {
			vector<double> row;
			row.push_back(launchDists[r + 1]);

			for (int s = 0; s < reefs.size(); s++) {
				double dist = sqrt(pow(reefs[r]->x - reefs[s]->x, 2) + pow(reefs[r]->y - reefs[s]->y, 2));
				row.push_back(dist);
			}
			row.push_back(retrieveDists[r + 1]);
			dMatrix.push_back(row);
		}
		dMatrix.push_back(retrieveDists);


		return dMatrix;
	}

private:
	static int count;
};

////////////////////////////////////////////////////////////////////////////

struct ClusterSoln {
public:
	ClusterSoln() : ID(count++) {}
	ClusterSoln(const vector<Cluster*>& clusters) : ID(count++), clusters(clusters) {}
	ClusterSoln(const vector<Cluster*>& clusters, const vector<vector<double>>& centroidMatrix) : 
		ID(count++), clusters(clusters), centroidMatrix(centroidMatrix) {}
	//typedef vector<ReefPt*> Cluster;

	const int ID;
	vector<Cluster*> clusters;
	vector<vector<double>> centroidMatrix;

	//void setClusters(vector<Cluster*>& clusters_in) {
	//	clusters = clusters_in;
	//}

private:
	static int count;
};	

//struct Route_MS {
//public:
//	Route_MS(vector<Cluster*> clustOrder) : ID(count++), clustOrder(clustOrder) {}
//	vector<Pt> getLaunchPts() {
//		vector<Pt> centroids;
//		for (auto& clust : clustOrder) {
//			centroids.push_back(clust->getCentroid(/*clust->getReefs()*/));
//		}
//		vector<Pt> launchPts;
//		for (int c = 0; c < centroids.size()-1; c++) {
//			Pt launchPt((centroids[c].getXY().first + centroids[c + 1].getXY().first) / 2, (centroids[c].getXY().second + centroids[c + 1].getXY().second) / 2);
//			launchPts.push_back(launchPt);
//		}
//		return launchPts;
//	}
//
//private:
//	static int count;
//	const int ID;
//	const vector<Cluster*> clustOrder;
//	//MS* ms;
//	const Pt depot;// ->? = problem.getDepot();
//	vector<Pt> launchPts = this->getLaunchPts();
//};

struct MSSoln {
public:
	//MSSoln(const Problem& inst) : ID(count++), inst(inst) {}
	MSSoln(const Problem* inst, ClusterSoln* clustSoln) :
		ID(count++), inst(inst), clustSoln(clustSoln), launchPts(clustSoln->clusters.size() + 1, nullptr) {}
	//MSSoln(const Problem& inst, vector<Cluster*> clustOrder/*, vector<Route_MS*> routes*/) :
	//	ID(count++), inst(inst), clustOrder(clustOrder)/*, routes(routes)*/{}

	const int ID;
	const Problem* inst;				// problem instance
	ClusterSoln* clustSoln;

	//vector<Cluster*> clustOrder;		// ordered clusters		////vector<int> clustOrder;		// ordered clusters by ID
	vector<Pt*> launchPts;

	//check this works correctly between correct points...
	vector<vector<double>> ordered_dMatrix() { 
		vector<vector<double>> dMatrix;
		vector<double> depotDists;
		depotDists.push_back(0.0);
		for (auto& clust : clustSoln->clusters) {
			double dist = sqrt(pow(inst->ms.depot.x - clust->getCentroid().x, 2) + pow(inst->ms.depot.y - clust->getCentroid().y, 2));
			depotDists.push_back(dist);
		}
		dMatrix.push_back(depotDists);
		// is this dangerous in case where order changes?!
		
		for (int c = 0; c < clustSoln->clusters.size(); c++) {
			vector<double> clustDists;
			clustDists.push_back(depotDists[c+1]);
			for (int d = 0; d < clustSoln->clusters.size(); d++) {
			double dist = sqrt(pow(clustSoln->clusters[c]->getCentroid().x - clustSoln->clusters[d]->getCentroid().x, 2) + pow(clustSoln->clusters[c]->getCentroid().y - clustSoln->clusters[d]->getCentroid().y, 2));
			clustDists.push_back(dist);
			}
			dMatrix.push_back(clustDists);
		}
		return dMatrix;
	}
	
	double getDist() {
		double dist = //calculatePtDistance(inst->ms.depot, clustOrder[0]->getCentroid());
			sqrt(pow(inst->ms.depot.x - launchPts[0]->x, 2) + pow(inst->ms.depot.y - launchPts[0]->y, 2));
		for (int c = 0; c < clustSoln->clusters.size(); c++) {
			dist += sqrt(pow(launchPts[c]->x - launchPts[c + 1]->x, 2) + pow(launchPts[c]->y - launchPts[c + 1]->y, 2));
				//calculatePtDistance(
				//	clustOrder[c]->getCentroid(), 
				//	clustOrder[c+1]->getCentroid()
				//);
		}
		dist += sqrt(pow(inst->ms.depot.x - launchPts[clustSoln->clusters.size()]->x, 2) + pow(inst->ms.depot.y - launchPts[clustSoln->clusters.size()]->y, 2));//calculatePtDistance(inst->ms.depot, clustOrder[clustOrder.size()-1]->getCentroid());
		//sqrt(pow(inst->ms.depot.x - launchPts[clustOrder.size() - 1]->x, 2) + pow(inst->ms.depot.y - launchPts[clustOrder.size() - 1]->y, 2));
		return dist;
	}

	vector<Pt*> getRoute() {			//update/check this! use mp's of centroids...
		vector<Pt*> route;//(launchPts.size() + 1, nullptr);
		route.push_back(new Pt(inst->ms.depot)); // Assuming Pt has a copy/move constructor
		for (auto& pt : launchPts) {
			route.push_back(pt);
		}
		route.push_back(new Pt(inst->ms.depot)); // Assuming Pt has a copy/move constructor
		return route;
	}
	//Cluster& getCluster(int ID) {
	//	for (auto& clust : clustOrder) {
	//		if (clust->getID() == ID) {
	//			return *clust;
	//		}
	//	}
	//}

	//int getClusterIndex(Cluster& cluster) {
	//	return cluster.getID() - 1;
	//}
private:
	static int count;
};


struct TenderSoln {
typedef vector<Pt*> Route_Tender;
public:
	//TenderSoln() : ID(count++), cluster(cluster), greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	TenderSoln(Cluster* cluster) : ID(count++), cluster(cluster) {}
	TenderSoln(Cluster* cluster, vector<Route_Tender>& routes) :
		ID(count++), cluster(cluster), routes(routes) {}//, greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	//TenderSoln(Cluster& cluster, vector<Route_Tender> routes, bool greedy, bool without_clust, bool within_clust, bool greedy_again) :
	//	ID(count++), cluster(cluster), routes(routes), greedy(greedy), without_clust(without_clust), within_clust(within_clust), greedy_again(greedy_again) {}

	const int ID;

	Cluster* cluster;
	vector<Route_Tender> routes;

private:
	static int count;
};

struct FullSoln {
public:
//FullSoln(Problem* dat, MSSoln* msSoln, TenderSoln* tenderSoln, vector<Cluster*> clusters) :
//		ID(count++), dat(dat), msSoln(msSoln), tenderSoln(tenderSoln), clusters(clusters) {}
FullSoln(Problem* dat, MSSoln* msSoln, ClusterSoln* clusterSolns, vector<TenderSoln*> tenderSoln) :
		ID(count++), dat(dat), msSoln(msSoln), clusterSolns(clusterSolns), tenderSolns(tenderSoln), 
		greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}

	int ID;
	const Problem* dat;
	ClusterSoln* clusterSolns;
	MSSoln* msSoln;
	vector<TenderSoln*> tenderSolns;
	//vector<Cluster*> clusters;

	const bool greedy;
	const bool without_clust;
	const bool within_clust;
	const bool greedy_again;

private:
	static int count;
};

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

struct SA {
	SA(int num_iterations, int initial_temp, double cooling_rate) :
		num_iterations(num_iterations), initial_temp(initial_temp), cooling_rate(cooling_rate) {};


	const int initial_temp;
	const double cooling_rate;
	const int num_iterations;
	//string file_name;
	vector<double> best_dist;
	vector<double> current_dist;
	vector<double> new_dist;
	vector<double> temp;

};

