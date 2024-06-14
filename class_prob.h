#pragma once
#include <chrono>
#include <ctime>

struct Pt {
public:
	const int ID;
	double x;
	double y;

	Pt(double x = 0.0, double y = 0.0) : ID(count++), x(x), y(y) {}	// Default constructor
	Pt(pair<double, double> coords) : ID(count++), x(coords.first), y(coords.second) {}

	// Copy constructor
	Pt(const Pt& other) : ID(other.ID), x(other.x), y(other.y) {}

	//destructor
	~Pt() {}

private:
	static int count;
};

vector<Pt> initReefs(int no_pts, int sol_space = 100) {
	vector<Pt> reefs;
	for (int i = 0; i < no_pts; i++) {
		Pt pt = Pt(rand() % sol_space, rand() % sol_space);
		reefs.push_back(pt);
	}
	return reefs;
}

struct MS {
public:
	const int ID;
	const int cap = NULL;
	Pt depot;

	MS() : ID(count++), cap(NULL) {}
	MS(Pt depot) : ID(count++), depot(depot) {}
	MS(int cap) : ID(count++), cap(cap) {}
	MS(int cap, Pt depot) : ID(count++), cap(cap), depot(depot) {}
private:
	static int count;
};

struct Drone {
public:
	const int ID;
	const int cap;

	Drone(int cap) : ID(count++), cap(cap)/*, route(route)*/ {}
private:
	static int count;
};

pair<double, double> normaliseWeights(pair<double, double> weights) {
	double totalWeight = weights.first + weights.second; // Assuming weights is a pair<double, double>
	if (totalWeight != 0.0) {	// Avoid division by zero
		weights.first /= totalWeight;
		weights.second /= totalWeight;
	}
	else (weights = make_pair(0.5, 0.5)); // Default weights (0.5, 0.5)
	return weights;
}
void normaliseWeights(double& weight_a, double& weight_b) {
	double totalWeight = weight_a + weight_b; // Assuming weights is a pair<double, double>
	if (totalWeight != 0.0) {	// Avoid division by zero
		weight_a /= totalWeight;
		weight_b /= totalWeight;
	}
	else {// Default weights (0.5, 0.5)
		weight_a = 0.5;
		weight_b = 0.5;
	}
	return;
}

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

public:
	const vector<Pt> reefs;
	const MS ms = this->setMS(noClust, depot);
	const vector<Drone> drones = this->setDrones(noDrones, dCap);
	pair<double, double> weights;
	const string time;
	int kMeansIters;

	Problem(vector<Pt> reefs, int noClust, int noDrones, int dCap, pair<double, double> weights, Pt depot, int kMeansIters) :
		reefs(move(reefs)),
		noClust(noClust),
		depot(depot),
		noDrones(noDrones), dCap(dCap),
		weights(normaliseWeights(weights)), kMeansIters(kMeansIters),
		time(getCurrentTime())
	{
		if (this->kMeansIters <= 0) this->kMeansIters = 1;
	}

	vector<Pt*> getReefPointers() const {
		vector<Pt*> reefPtrs;
		for (auto& reef : reefs) {
			reefPtrs.push_back(const_cast<Pt*>(&reef));
		}
		return reefPtrs;
	}
	int getnumClusters() const { return noClust; }
	int getnumDrones() const { return noDrones; }
	int get_dCap() const { return dCap; }
	Pt getDepot() const { return depot; }
	vector<vector<double>> getDMatrix() {
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
bool checkParameters(const Problem& instance) {
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
Problem CreateInst(int no_pts = 48,/*100;*/int noClust = 4,
	int noDrones = 4, int dCap = 3,
	pair<double, double> weights = make_pair(1, 1),
	Pt depot = Pt(0, 0), int kMeansIters = pow(10, 2)) {
	Problem inst = Problem(initReefs(no_pts), noClust, noDrones, dCap, weights, depot, kMeansIters);
	if (checkParameters(inst)) return inst;
}

Problem CreateInst(const Problem& inst_ex, int noClust, int noDrones, int dCap) {
	Problem inst = Problem(inst_ex.reefs, noClust, noDrones, dCap, inst_ex.weights, inst_ex.getDepot(), inst_ex.kMeansIters);
	if (checkParameters(inst)) return inst;
}
Problem CreateInst(const Problem& inst_ex, pair<double, double> weights) {
	Problem inst = Problem(inst_ex.reefs, inst_ex.getnumClusters(), inst_ex.getnumDrones(), inst_ex.get_dCap(), weights, inst_ex.getDepot(), inst_ex.kMeansIters);
	if (checkParameters(inst)) return inst;
}

void printSetup(const Problem& inst) {
	printf("Problem inst:\n");
	printf("Depot:\t\t\t(%.2f, %.2f)\n", inst.getDepot().x, inst.getDepot().y);
	printf("Number of reefs: \t%d\n", inst.reefs.size());
	printf("Number of clusters:\t%d\n", inst.getnumClusters());
	printf("Number of drones:\t%d\n", inst.getnumDrones());
	printf("Drone capacity:\t\t%d\n", inst.get_dCap());
	printf("\t\t\tWeighting:\n\t\t\tMS: %.3f\t\tDrone %.3f\n", inst.weights.first, inst.weights.second);
	printf("Time initialised: \t");
}
