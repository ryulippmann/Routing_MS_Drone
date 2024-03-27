#pragma once
#include <chrono>
#include <ctime>

//#ifndef calcs_h
//#define calcs_h

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

vector<Pt> initReefs(int no_pts = 100, int sol_space = 100) {
	vector<Pt> reefs;
	for (int i = 0; i < no_pts; i++) {
		Pt pt = Pt(rand() % sol_space, rand() % sol_space);
		reefs.push_back(pt);
	}//for(pts) //cout << "\n";
	return reefs;
}

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
	const int tenderCap;

	MS setMS(int msCap, Pt depot) {
		MS ms(msCap, depot);
		return ms;
	}

	vector<Tender> setTenders(int numTenders, int tenderCap) {
		vector<Tender> tenders;
		for (int i = 0; i < numTenders; ++i) { tenders.push_back(Tender(tenderCap)); }
		return tenders;
	}

	string getCurrentTime() const {
		auto now = chrono::system_clock::now();
		time_t time = chrono::system_clock::to_time_t(now);
		tm localTime;                       // Convert time to local time
		localtime_s(&localTime, &time);
		char output[80];
		//string output;
		strftime(output, sizeof(output), "%y-%m-%d_%H-%M-%S", &localTime);
		return string(output);
	}

public:
	Problem(vector<Pt> reefs, int numClust, Pt depot, int numTenders, int tenderCap, pair<double, double> weights, int kMeansIters) :
		reefs(move(reefs)),
		numClust(numClust), 
		depot(depot), 
		numTenders(numTenders), tenderCap(tenderCap),
		weights(weights), kMeansIters(kMeansIters),
		time(getCurrentTime()) 
	{
		//auto now = chrono::system_clock::now();
		//time_t time = chrono::system_clock::to_time_t(now);
		//tm localTime;                       // Convert time to local time
		//char output[80];
		//strftime(output, sizeof(output), "%y-%m-%d_%H-%M-%S", &localTime);
		//time = output;
	}
	
	//const vector<vector<double>> dMatrix = this->getDMatrix(reefs, depot);
	const vector<Pt> reefs;
	const MS ms						= this->setMS(numClust, depot);
	const vector<Tender> tenders	= this->setTenders(numTenders, tenderCap);
	const pair<double, double> weights;
	const string time;
	const int kMeansIters;//1000

	//void printSetup() {
	//	printf("Problem inst:\n");
	//	printf("Number of reefs: %.0f\n", reefs.size());
	//	printf("Number of clusters: %d\n", numClust);
	//	printf("Depot: (%.2f, %.2f)\n", depot.x, depot.y);
	//	printf("Number of tenders: %d\n", numTenders);
	//	printf("Tender capacity: %d\n", tenderCap);
	//	printf("Weighting:\n\t\tMS: %.1f\tDrone %.1f\n", weights.first, weights.second);
	//}

	vector<Pt*> getReefPointers() const { 
		vector<Pt*> reefPtrs;
		for (auto& reef : reefs) {
			reefPtrs.push_back(const_cast<Pt*>(&reef));
		}
		return reefPtrs; 
	}
	int getnumClusters()	const	{ return numClust; }
	int getnumTenders()		const	{ return numTenders; }
	int getTenderCap()		const	{ return tenderCap; }
	Pt getDepot()			const	{ return depot; }
	vector<vector<double>> getDMatrix(/*vector<Pt> reefs, Pt depot*/) {
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
	
};

/// <summary>
/// Check if the parameters are valid: return true=valid, false=not
/// </summary>
/// <param name="instance"></param>
/// <returns></returns>
bool checkParameters(Problem instance) {
	try { if (instance.getnumClusters() * instance.getnumTenders() * instance.getTenderCap() != instance.getReefPointers().size()) throw invalid_argument("numClust * numTenders * tenderCap != no_pts"); }
	catch (const invalid_argument& e) { cout << endl << e.what() << endl; return false; }
	return true;
}

/// <summary>
/// validity check later: (numClust * numTenders * tenderCap = no_pts)
/// </summary>
/// <param name="no_pts"></param>
/// <param name="numClust"></param>
/// <param name="numTenders"></param>
/// <param name="tenderCap"></param>
/// <param name="w_ms"></param>
/// <param name="w_d"></param>
/// <returns></returns>
Problem CreateInst(	int no_pts = 48,/*100;*/int numClust = 4, /*5;*/
					int numTenders = 4,		int tenderCap = 3,
					double w_ms = 1,		double w_d = 1,
					Pt depot = Pt(0,0),		int kMeansIters = pow(10,4))	{
	Problem inst = Problem(initReefs(no_pts), numClust, depot, numTenders, tenderCap, make_pair(w_ms, w_d), kMeansIters);
		//if (checkParameters(inst) == false) return 0;		// check if parameters are valid
		//else {return inst;
	if (checkParameters(inst)) return inst;
}
	//inst(initReefs(no_pts), numClust, depot, numTenders, tenderCap, make_pair(w_ms, w_d));

void printSetup(Problem inst) {
	printf("Problem inst:\n");
	printf("Number of reefs: \t%d\n", inst.reefs.size());
	printf("Number of clusters:\t%d\n", inst.getnumClusters());
	printf("Depot:\t\t\t(%.2f, %.2f)\n", inst.getDepot().x, inst.getDepot().y);
	printf("Number of tenders:\t%d\n", inst.getnumTenders());
	printf("Tender capacity:\t%d\n", inst.getTenderCap());
	printf("\t\t\tWeighting:\n\t\t\tMS: %.1f\t\tDrone %.1f\n", inst.weights.first, inst.weights.second);
	printf("Time initialised: \t");
}


////////////////////////////////////////////////////////////////////////////
