#pragma once
using namespace std;
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

struct ReefPt {
	ReefPt(int ID, double x, double y) : ID(ID), x(x), y(y) {}

	const int ID;
	const double x;
	const double y;

	pair<double, double> getXY() {
		return make_pair(x, y);
	}
	//void setXY(double x, double y) {
	//	this->x = x;
	//	this->y = y;
	//}
};

struct Pt {
public:
	Pt(int ID, double x = 0.0, double y = 0.0) : ID(ID), x(x), y(y) {}
	Pt(int ID, pair<double, double> coords) : ID(ID), x(coords.first), y(coords.second) {}

	pair<double, double> getXY() {
		return make_pair(x, y);
	}

private:
	const int ID;
	double x;
	double y;
};

struct MS {
	const int ID;
	const int cap;
	Pt depot;
};

struct Tender {
	int ID;
	int cap;
};

struct Problem {
	vector<ReefPt> reefs;
	vector<vector<double>> dMatrix;
	MS ms;
	vector<Tender> tenders;
	int numClust;
	int numTenders;
};

struct Cluster {
	int ID;
	vector<ReefPt> reefs;
	//Pt centroid;
	Pt start;
	Pt end;

	Pt getCentroid() {
		double x=0.0;
		double y=0.0;
		for (auto& reef : reefs) {
			pair<double,double> coords = reef.getXY();
			x += coords.first;
			y += coords.second;
		}
		Pt centroid(x/reefs.size(), y/reefs.size());
		return centroid;
	}
};

struct Route_MS {
	int ID;
	vector<Cluster*> clustOrder;

};

struct MSSoln {
	int ID;
	Problem* inst;
	//MS ms;
	vector<Cluster*> clustOrder;		// ordered clusters
	vector<Route_MS*> routes;			// routes for each cluster

	const bool greedy;
	const bool without_clust;
	const bool within_clust;
	const bool greedy_again;

	vector<vector<double>> dMatrix() {
		vector<vector<double>> dMatrix;
		pair<double, double> depotCoords = inst->ms.depot.getXY();//ms.depot.getXY();
		vector<double> depotDists;
		depotDists.push_back(0.0);
		for (auto& clust : clustOrder) {
			pair<double, double> clustCoords = clust->getCentroid().getXY();
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
				pair<double, double> reefCoords = reef.getXY();
				double dist = sqrt(pow(depotCoords.first - reefCoords.first, 2) + pow(depotCoords.second - reefCoords.second, 2));
				clustDists.push_back(dist);
			}
			dMatrix.push_back(clustDists);
		}
		return dMatrix;
	}
};

struct Route_Tender {
	int ID;
	vector<ReefPt*> reefs;
};

struct TenderSoln {
	int ID;
	vector<Route_Tender*> routes;
	Cluster& cluster;
	const bool greedy;
	const bool without_clust;
	const bool within_clust;
	const bool greedy_again;

};

struct FullSoln {
	int ID;
	Problem dat;
	MSSoln msSoln;
	TenderSoln tenderSoln;
	vector<Cluster*> clusters;

};

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

