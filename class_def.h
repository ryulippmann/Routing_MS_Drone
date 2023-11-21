#pragma once
using namespace std;
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

struct ReefPt {
public:
	ReefPt(double x, double y) : ID(count++), x(x), y(y) {}
	ReefPt(pair<double, double> coords) : ReefPt(coords.first, coords.second) {}

	//int getID() const { return ID; }
	const int ID;
	const double x;
	const double y;

	pair<double, double> getXY() const { return make_pair(x, y); }

	//ReefPt& operator=(const ReefPt& point) {	// Assignment operator
	//	//pair<double, double> coords = point.getXY();
	//	if (this != &point) { ReefPt(point.x, point.y); }
	//	return *this;	// self-assignment check	
	//}

private:
	static int count;
};

struct Pt {
public:
	Pt(double x = 0.0, double y = 0.0) : ID(count++), x(x), y(y) {}	// Default constructor
	Pt(pair<double, double> coords) : ID(count++), x(coords.first), y(coords.second) {}

	const int ID;
	const double x;
	const double y;

	pair<double, double> getXY() const { return make_pair(x, y); }

	//Pt& operator=(const Pt& point) {	// Assignment operator
	//	pair<double, double> coords = point.getXY();
	//	if (this != &point) { Pt (coords.first, coords.second); }
	//	return *this;	// self-assignment check
	//}

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
public:
	Problem(vector<ReefPt> reefs, int numClust, Pt depot, int numTenders, int tenderCap) :
		reefs(reefs), numClust(numClust), depot(depot), numTenders(numTenders), tenderCap(tenderCap)
		//ms(MS(numClust)), msCap(msCap), tenderCap(tenderCap), numTenders(numTenders)
	{}
	//Problem(vector<ReefPt> reefs, int numClust, Pt depot, int numTenders, int tenderCap) :
	//	reefs(reefs), numClust(numClust), depot(depot), numTenders(numTenders), tenderCap(tenderCap),
	//	ms(MS())//, msCap(msCap), tenderCap(tenderCap), numTenders(numTenders)
	//{}

	vector<ReefPt*> getReefPointers() const { 
		vector<ReefPt*> reefPtrs;
		for (auto& reef : reefs) {
			reefPtrs.push_back(const_cast<ReefPt*>(&reef));
		}
		return reefPtrs; 
	}
	int getnumClusters() const { return numClust; }
	Pt getDepot() const { return depot; }
	MS getMS() const { return ms; }
	int getnumTenders() const { return numTenders; }
	vector<Tender> getTenders() const { return tenders; }

	vector<vector<double>> getDMatrix(vector<ReefPt> reefs, Pt depot) {
		vector<vector<double>> dMatrix;
		pair<double, double> depotCoords = depot.getXY();
		vector<double> depotDists;
		depotDists.push_back(0.0);
		for (auto& reef : reefs) {
			pair<double, double> reefCoords = reef.getXY();
			double dist = sqrt(pow(depotCoords.first - reefCoords.first, 2) + pow(depotCoords.second - reefCoords.second, 2));
			depotDists.push_back(dist);
		}
		dMatrix.push_back(depotDists);
		int r = 0;
		for (auto& reef_a : reefs) {
			r++;
			vector<double> row;
			row.push_back(depotDists[r]);
			for (auto& reef_b : reefs) {
				pair<double, double> coords_a = reef_a.getXY();
				pair<double, double> coords_b = reef_b.getXY();
				double dist = sqrt(pow(coords_a.first - coords_b.first, 2) + pow(coords_a.second - coords_b.second, 2));
				row.push_back(dist);
			}
			dMatrix.push_back(row);
		}
		return dMatrix;
	}

private:
	vector<Tender> setTenders(int numTenders, int tenderCap) {
		vector<Tender> tenders;
		for (int i = 0; i < numTenders; ++i) { tenders.push_back(Tender(tenderCap)); }
		return tenders;
	}

	MS setMS(int msCap, Pt depot) {
		MS ms(msCap, depot);
		return ms;
	}

	const vector<ReefPt> reefs;
	const vector<vector<double>> dMatrix = this->getDMatrix(reefs, depot);

	const int numClust;
	const Pt depot;
	const MS ms = this->setMS(numClust, depot);

	const int tenderCap;
	const int numTenders;
	const vector<Tender> tenders = this->setTenders(numTenders, tenderCap);
};

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

struct Cluster {
public:
	Cluster(vector<ReefPt*> reefs/*, Pt start, Pt end*/) : ID(count++), reefs(reefs)/*, start(start), end(end)*/ {}

	const int ID;
	const vector<ReefPt*> reefs;

	Pt getCentroid(/*vector<ReefPt> reefs*/) {
		double x=0.0;
		double y=0.0;
		for (auto& reef : reefs/*this->getReefs()*/) {
			x += reef->x;
			y += reef->y;
		}
		Pt centroid(x/reefs.size(), y/reefs.size());
		return centroid;
	}

private:
	static int count;
	//Pt centroid = this->getCentroid(/*reefs*/);
};

////////////////////////////////////////////////////////////////////////////

struct ClusterSoln {
public:
	ClusterSoln() : ID(count++) {}
	ClusterSoln(const vector<Cluster*> clusters) : ID(count++), clusters(clusters) {}

	const int ID;
	const vector<Cluster*> clusters;

	//void setClusters(vector<Cluster*>& clusters_in) {
	//	clusters = clusters_in;
	//}

	//vector<Tender*> setTenders(vector<Tender*> tenders) {
	//	vector<Tender*> tenders_out;
	//	for (auto& tender : tenders) {
	//		tenders_out.push_back(tender);
	//	}
	//	return tenders_out;
	//}

private:
	static int count;
};


struct TenderSoln {
typedef vector<ReefPt*> Route_Tender;
public:
	TenderSoln() : ID(count++), cluster(cluster), greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	TenderSoln(Cluster& cluster, vector<Route_Tender> routes) : ID(count++), cluster(cluster), routes(routes), greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	TenderSoln(Cluster& cluster, vector<Route_Tender> routes, bool greedy, bool without_clust, bool within_clust, bool greedy_again) :
		ID(count++), cluster(cluster), routes(routes), greedy(greedy), without_clust(without_clust), within_clust(within_clust), greedy_again(greedy_again) {}

	const int ID;

	Cluster& cluster;
	vector<Route_Tender> routes;

	const bool greedy;
	const bool without_clust;
	const bool within_clust;
	const bool greedy_again;

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
	MSSoln(const Problem& inst, vector<Cluster*> clustOrder/*, vector<Route_MS*> routes*/) :
		ID(count++), inst(inst), clustOrder(clustOrder)/*, routes(routes)*/{}

	const int ID;
	const Problem& inst;				// problem instance
	vector<Cluster*> clustOrder;		// ordered clusters
	////vector<int> clustOrder;		// ordered clusters by ID
	vector<vector<double>> dMatrix() {
		vector<vector<double>> dMatrix;
		pair<double, double> depotCoords = inst.getMS().depot.getXY();//ms.depot.getXY();
		vector<double> depotDists;
		depotDists.push_back(0.0);
		for (auto& clust : clustOrder) {
			pair<double, double> clustCoords = clust->getCentroid(/*clust->getReefs()*/).getXY();
			double dist = sqrt(pow(depotCoords.first - clustCoords.first, 2) + pow(depotCoords.second - clustCoords.second, 2));
			depotDists.push_back(dist);
		}
		dMatrix.push_back(depotDists);
		// is this dangerous in case where order changes?!
		int c = 0;
		for (auto& clust : clustOrder) {
			c++;
			vector<double> clustDists;
			clustDists.push_back(depotDists[c]);
			for (auto& reef : clust->reefs) {
				pair<double, double> reefCoords = reef->getXY();
				double dist = sqrt(pow(depotCoords.first - reefCoords.first, 2) + pow(depotCoords.second - reefCoords.second, 2));
				clustDists.push_back(dist);
			}
			dMatrix.push_back(clustDists);
		}
		return dMatrix;
	}

	vector<Pt> getRoute() {			//update/check this! use mp's of centroids...
		vector<Pt> route;
		route.push_back(inst.getMS().depot);
		for (auto& clust : clustOrder) {
			route.push_back(clust->getCentroid());
		}
		route.push_back(inst.getMS().depot);
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

struct FullSoln {
public:
//FullSoln(Problem* dat, MSSoln* msSoln, TenderSoln* tenderSoln, vector<Cluster*> clusters) :
//		ID(count++), dat(dat), msSoln(msSoln), tenderSoln(tenderSoln), clusters(clusters) {}

FullSoln(Problem dat, MSSoln msSoln, ClusterSoln clusterSolns, vector<TenderSoln> tenderSoln) :
		ID(count++), dat(dat), msSoln(msSoln), clusterSolns(clusterSolns), tenderSolns(tenderSoln) {}

	int ID;
	const Problem& dat;
	ClusterSoln clusterSolns;
	MSSoln msSoln;
	vector<TenderSoln> tenderSolns;
	//vector<Cluster*> clusters;
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

