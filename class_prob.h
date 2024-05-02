#pragma once
#include <chrono>
#include <ctime>

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

private:
	static int count;
};

struct Drone {
public:
	Drone(int cap) : ID(count++), cap(cap)/*, route(route)*/ {}

	const int ID;
	const int cap;
private:
	static int count;
};

const struct Problem {
private:
	const int noClust;
	const Pt depot;
	const int noDrones;
	const int dCap;

	MS setMS(int msCap, Pt depot) {
		MS ms(msCap, depot);
		return ms;
	}

	vector<Drone> setDrones(int noDrones, int dCap) {
		vector<Drone> drones;
		for (int i = 0; i < noDrones; ++i) { drones.push_back(Drone(dCap)); }
		return drones;
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

	pair<double, double> normaliseWeights(pair<double, double> weights) {
		double totalWeight = weights.first + weights.second; // Assuming weights is a pair<double, double>

		// Check if totalWeight is not zero to avoid division by zero
		if (totalWeight != 0.0) {
			weights.first /= totalWeight;
			weights.second /= totalWeight;
		}
		return weights;
	}

public:
	Problem(vector<Pt> reefs, int noClust, Pt depot, int noDrones, int dCap, pair<double, double> weights, int kMeansIters) :
		reefs(move(reefs)),
		noClust(noClust), 
		depot(depot), 
		noDrones(noDrones), dCap(dCap),
		weights(normaliseWeights(weights)), kMeansIters(kMeansIters),
		time(getCurrentTime()) 
	{
		//auto now = chrono::system_clock::now();
		//time_t time = chrono::system_clock::to_time_t(now);
		//tm localTime;                       // Convert time to local time
		//char output[80];
		//strftime(output, sizeof(output), "%y-%m-%d_%H-%M-%S", &localTime);
		//time = output;
	}
	
	const vector<Pt> reefs;
	const MS ms					= this->setMS(noClust, depot);
	const vector<Drone> drones	= this->setDrones(noDrones, dCap);
	pair<double, double> weights;// = this->normaliseWeights(weights);
	const string time;
	const int kMeansIters;//1000

	vector<Pt*> getReefPointers() const { 
		vector<Pt*> reefPtrs;
		for (auto& reef : reefs) {
			reefPtrs.push_back(const_cast<Pt*>(&reef));
		}
		return reefPtrs; 
	}
	int getnumClusters()	const	{ return noClust; }
	int getnumDrones()		const	{ return noDrones; }
	int get_dCap()		const	{ return dCap; }
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
	try { if (instance.getnumClusters() * instance.getnumDrones() * instance.get_dCap() != instance.getReefPointers().size()) throw invalid_argument("noClust * noDrones * dCap != no_pts"); }
	catch (const invalid_argument& e) { cout << endl << e.what() << endl; return false; }
	return true;
}

/// <summary>
/// validity check later: (noClust * noDrones * dCap = no_pts)
/// </summary>
/// <param name="no_pts">= 48</param>
/// <param name="noClust">= 4</param>
/// <param name="noDrones">= 4</param>
/// <param name="dCap">= 3</param>
/// <param name="weights">= (1,1)</param>
/// <param name="depot">= Pt(0,0)</param>
/// <param name="kMeansIters">= 100</param>
Problem CreateInst(	int no_pts = 48,/*100;*/int noClust = 4, /*5;*/
					int noDrones = 4,		int dCap = 3,
					pair<double, double> weights = make_pair(1,1), //= make_pair(1,1)
					Pt depot = Pt(0,0),		int kMeansIters = pow(10,2))	{
		Problem inst = Problem(initReefs(no_pts), noClust, depot, noDrones, dCap, weights, kMeansIters);
		if (checkParameters(inst)) return inst;
}

Problem CreateInst(Problem inst_ex, int noClust, int noDrones, int dCap) {
	Problem inst = Problem(inst_ex.reefs, noClust, inst_ex.getDepot(), noDrones, dCap, inst_ex.weights, inst_ex.kMeansIters);
		if (checkParameters(inst)) return inst;
}

Problem CreateInst(Problem inst_ex, pair<double, double> weights) {
	Problem inst = Problem(inst_ex.reefs, inst_ex.getnumClusters(), inst_ex.getDepot(), inst_ex.getnumDrones(), inst_ex.get_dCap(), weights, inst_ex.kMeansIters);
		if (checkParameters(inst)) return inst;
}

void printSetup(Problem inst) {
	printf("Problem inst:\n");
	printf("Depot:\t\t\t(%.2f, %.2f)\n", inst.getDepot().x, inst.getDepot().y);
	printf("Number of reefs: \t%d\n", inst.reefs.size());
	printf("Number of clusters:\t%d\n", inst.getnumClusters());
	printf("Number of drones:\t%d\n", inst.getnumDrones());
	printf("Drone capacity:\t\t%d\n", inst.get_dCap());
	printf("\t\t\tWeighting:\n\t\t\tMS: %.1f\t\tDrone %.1f\n", inst.weights.first, inst.weights.second);
	printf("Time initialised: \t");
}


////////////////////////////////////////////////////////////////////////////
