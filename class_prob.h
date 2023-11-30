#pragma once
using namespace std;
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
//#include "MSSoln.h"
//class MSSoln;
// 
//#include "calcs.h"

//#ifndef calcs_h
//#define calcs_h

//#include "calcs.h"

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

	// Copy constructor
	Pt(const Pt& other) : ID(other.ID), x(other.x), y(other.y) {}
	
	const int ID;
	const double x;
	const double y;
	//pair<double, double> getXY() const { return make_pair(x, y); }

	//static Pt* getPtByID(int targetID);
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
	int getnumClusters() const { return numClust; }
	int getnumTenders() const { return numTenders; }
	vector<vector<double>> getDMatrix(vector<Pt> reefs, Pt depot) {
		vector<vector<double>> dMatrix;
		vector<double> depotDists;
		depotDists.push_back(0.0);					// depot to itself = 0
		for (auto& reef : reefs) {					// depot to each reef
			double dist = sqrt(pow(depot.x - reef.x, 2) + pow(depot.y - reef.y, 2));
			depotDists.push_back(dist);				// dist depot to each reef
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

	const int tenderCap;
	const vector<Tender> tenders = this->setTenders(numTenders, tenderCap);
};

////////////////////////////////////////////////////////////////////////////
