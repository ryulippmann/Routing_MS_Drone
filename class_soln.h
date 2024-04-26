#pragma once
#include "class_SA.h"


struct ClusterSoln {
public:
	ClusterSoln() : ID(count++) {}
	ClusterSoln(const vector<Pt*> reefs) : ID(count++), reefs(reefs) {}
	
	// Copy constructor for deep copy
	ClusterSoln(const ClusterSoln& other) :
		ID(other.ID), /*INST(other.INST), */reefs() {
		// Copy new Pt objects in the reefs vector
		for (auto& reef : other.reefs) {
			this->reefs.push_back(new Pt(*reef));
		}
	}

	const int ID;
	/*const*/ vector<Pt*> reefs;

	vector<vector<double>> calc_centMatrix(const vector<ClusterSoln*>& clusters, const Pt depot) {
		vector<vector<double>> centroidMatrix;
		//centroids.insert(centroids.begin(), ClusterPoint(depot.first, depot.second));
		vector<double> depot_row;
		depot_row.push_back(0);
		for (int i = 0; i < clusters.size(); i++) {		// for each cluster
			depot_row.push_back(calculatePtDistance(depot, clusters[i]->getCentroid()));
		}
		centroidMatrix.push_back(depot_row);

		for (int i = 0; i < clusters.size(); i++) {		// for each cluster
			vector<double> row(clusters.size() + 1);
			row[0] = depot_row[i + 1];
			for (int j = 0; j < clusters.size(); j++) {	// for each cluster
				row[j + 1] = calculatePtDistance(clusters[i]->getCentroid(), clusters[j]->getCentroid());
			}
			centroidMatrix.push_back(row);
		}

		return centroidMatrix;
	}

	Pt getCentroid() const {
		double x = 0.0;
		double y = 0.0;
		for (auto& reef : reefs) {
			x += reef->x;
			y += reef->y;
		}
		Pt centroid(x / reefs.size(), y / reefs.size());
		return centroid;
	}

	// includes launchpts and free link back to launchpt
	vector<vector<double>> getdMatrix(pair <Pt*, Pt*> launchpts) const {	//mothership, cluster
		vector<double> launchDists{ 0.0 };			// launchpt to itself = 0
		for (auto& reef : reefs) {					// launchpt to each reef
			double dist = sqrt(pow(reef->x - launchpts.first->x, 2) + pow(reef->y - launchpts.first->y, 2));
			launchDists.push_back(dist);			// dist launchpt to each reef
		} launchDists.push_back(DBL_MAX);			// launchpt to retrieve pt = inf

		vector<vector<double>> dMatrix;
		dMatrix.push_back(launchDists);				// launchpt to each reef

		vector<double> retrieveDists;				// retrieve pt to each reef
		retrieveDists.push_back(0.0);				// retrieve pt to itself = 0
		for (auto& reef : reefs) {					// retrieve pt to each reef
			double dist = sqrt(pow(reef->x - launchpts.second->x, 2) + pow(reef->y - launchpts.second->y, 2));
			retrieveDists.push_back(dist);			// dist retrieve pt to each reef
		} retrieveDists.push_back(0.0);				// FREE-LINK: retrieve pt return to launchpt

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

	// Copy assignment operator (for deep copy?)
	ClusterSoln& operator=(const ClusterSoln& other) {
		if (this != &other) {  // Check for self-assignment
			// Clean up existing reefs
			for (auto& reef : reefs) { delete reef; }		//careful with delete
			reefs.clear();
			// Copy new Pt objects in the reefs vector
			for (auto& reef : other.reefs) {
				this->reefs.push_back(new Pt(*reef));
			}
		}
		return *this;
	}

private:
	static int count;
};

struct MSSoln {
public:
	MSSoln(vector<ClusterSoln*> clustSolns) :
		ID(count++), clusters(clustSolns), launchPts(clustSolns.size() + 1, nullptr) {}
	// Copy constructor for deep copy
	MSSoln(const MSSoln& other) :
		ID(count++), clusters(), launchPts() {
		// Copy new ClusterSoln objects
		for (auto& cluster : other.clusters) {
			this->clusters.push_back(new ClusterSoln(*cluster)); 
		}
		for (auto* pt : other.launchPts) {
			this->launchPts.push_back(pt);
		}
	}
	MSSoln(vector<ClusterSoln*> clustSolns, vector<Pt*> launchPts) :
		ID(count++), clusters(clustSolns), launchPts() {
		// Create new DroneSoln objects with new memory locations for pointers
		for (auto* pt : launchPts) {
			this->launchPts.push_back(new Pt(*pt));
		}
	}

	const int ID;
	vector<ClusterSoln*> clusters;
	vector<Pt*> launchPts;

	//check this works correctly between correct points...
	vector<vector<double>> ordered_dMatrix() {
		vector<vector<double>> dMatrix;
		vector<double> depotDists;
		depotDists.push_back(0.0);
		for (auto& clust : clusters) {
			double dist = sqrt(pow(INST.ms.depot.x - clust->getCentroid().x, 2) + pow(INST.ms.depot.y - clust->getCentroid().y, 2));
			depotDists.push_back(dist);
		}
		dMatrix.push_back(depotDists); // is this dangerous in case where order changes?!

		for (int c = 0; c < clusters.size(); c++) {
			vector<double> clustDists;
			clustDists.push_back(depotDists[c + 1]);
			for (int d = 0; d < clusters.size(); d++) {
				double dist = sqrt(pow(clusters[c]->getCentroid().x - clusters[d]->getCentroid().x, 2) + pow(clusters[c]->getCentroid().y - clusters[d]->getCentroid().y, 2));
				clustDists.push_back(dist);
			}
			dMatrix.push_back(clustDists);
		}
		return dMatrix;
	}

	vector<vector<double>> launchPt_dMatrix() {
		vector<vector<double>> dMatrix;
		vector<double> depotDists;
		depotDists.push_back(0.0);
		for (auto& launch : launchPts) {
			double dist = sqrt(pow(INST.ms.depot.x - launch->x, 2) + pow(INST.ms.depot.y - launch->y, 2));
			depotDists.push_back(dist);
		}
		dMatrix.push_back(depotDists);

		for (int s = 0; s < launchPts.size(); s++) {
			vector<double> launchDists;
			launchDists.push_back(depotDists[s + 1]);
			for (int d = 0; d < launchPts.size(); d++) {
				double dist = sqrt(pow(launchPts[s]->x - launchPts[d]->x, 2) + pow(launchPts[s]->y - launchPts[d]->y, 2));
				launchDists.push_back(dist);
			}
			dMatrix.push_back(launchDists);
		}
		return dMatrix;
	}

	// warning: launchPts need to be updated before this is called
	// dist returned is the total return dist from depot to launchPts, not centroids!
	double getDist(bool print=false) const {
		if (launchPts.size() == 0) {
			throw runtime_error("Launch points not set!");
			return -1;
		}
		double dist = 0;
		double leg = sqrt(pow(INST.ms.depot.x - launchPts[0]->x, 2) + pow(INST.ms.depot.y - launchPts[0]->y, 2));
		dist += leg;
		if (print) printf("\nMS Dist: \t\t%.2f\t", leg);
		for (int c = 0; c < clusters.size(); c++) { 
			leg = sqrt(pow(launchPts[c]->x - launchPts[c + 1]->x, 2) + pow(launchPts[c]->y - launchPts[c + 1]->y, 2)); 
			dist += leg;
			if (print) printf("+  %.2f\t", leg);
		}
		leg = sqrt(pow(INST.ms.depot.x - launchPts[clusters.size()]->x, 2) + pow(INST.ms.depot.y - launchPts[clusters.size()]->y, 2));
		dist += leg;
		if (print) printf("+ %.2f\t\nTotal Dist: \t%.2f\n", leg, dist);
		return dist;
	}

	vector<Pt*> getRoute() {			//update/check this! use mp's of centroids...
		vector<Pt*> route;//(launchPts.size() + 1, nullptr);
		route.push_back(new Pt(INST.ms.depot)); // Assuming Pt has a copy/move constructor
		for (auto& pt : launchPts) {
			route.push_back(pt);
		}
		route.push_back(new Pt(INST.ms.depot)); // Assuming Pt has a copy/move constructor
		return route;
	}

	// Copy assignment operator for (deep copy?)
	MSSoln& operator=(const MSSoln/*&*/ other) {
		if (this != &other) {
			// Deep copy clusters
			for (auto& cluster : clusters) { delete cluster; }		//careful with delete
			clusters.clear();
			for (auto& cluster : other.clusters) {
				clusters.push_back(new ClusterSoln(*cluster));
			}

			launchPts = other.launchPts;
		}
		return *this;
	}

private:
	static int count;
};

struct DroneSoln {		//typedef vector<Pt*> Route_Drone;
public:
	DroneSoln(ClusterSoln cluster) : ID(count++), cluster(cluster) {}
	DroneSoln(ClusterSoln cluster, vector<vector<Pt*>> routes, pair<Pt*, Pt*> launchPts) :
		ID(count++), cluster(cluster), routes(routes), launchPts(launchPts) {}//, greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}

	// Copy constructor for deep copy
	DroneSoln(const DroneSoln& other, bool reef_copy = false) :
		ID(other.ID), cluster(other.cluster), routes(), launchPts(other.launchPts) {
		//if (!reef_copy) { 
		//	this->cluster = other.cluster; 
		//}
		//else {
		//	this->cluster = nullptr;
		//}
		// Copy new route vectors		
		if (!reef_copy) {
			for (auto& route : other.routes) {
				vector<Pt*> newRoute;
				for (auto& pt : route) { newRoute.push_back(new Pt(*pt)); }
				this->routes.push_back(newRoute);
			}
		}
	}

	const int ID;
	ClusterSoln cluster;
	vector<vector<Pt*>> routes;
	pair<Pt*, Pt*> launchPts;

	// route dist for cluster c		// this is ONLY called in greedyDroneCluster() && initDroneSoln()
	double getDroneRouteDist(int c = -1) const {
		if (c == -1) {
			double dist = 0;
			for (int c = 0; c < routes.size(); c++) { dist += getDroneRouteDist(c); }
			return dist;
		}
		vector<Pt*> route = routes[c];
		double route_dist = 0;
		vector<Pt*> reefs;
		for (auto& pt : cluster.reefs) { reefs.push_back(pt); }
		reefs.insert(reefs.begin(), launchPts.first);
		reefs.push_back(launchPts.second);

		vector<vector<double>> dMatrix = cluster.getdMatrix(launchPts);
		for (int i = 0; i < route.size() - 2; ++i) {			// -2 to exclude return trip to launchPt
			int u = findIndexByID(route[i]->ID,		reefs) - 1;
			int v = findIndexByID(route[i + 1]->ID, reefs) - 1;
			route_dist += dMatrix[u][v];
		}
		return route_dist;
	}
	// route dist for given route	// this is called by FullSoln.getTotalDist()
	double getDroneRouteDist(vector<Pt*> route) const {
		double route_dist = 0;
		for (int i = 0; i < route.size() - 2; ++i) {			// -2 to exclude return trip to launchPt	
			route_dist += calculatePtDistance(route[i], route[i + 1]);
		}
		return route_dist;
	}
	
	// Copy assignment operator for deep copy
	DroneSoln& operator=(const DroneSoln& other) {
		if (this != &other) {
			//ID = other.ID;
			cluster = other.cluster;
			launchPts = other.launchPts;

			// Clear existing routes
			for (auto& route : routes) {
				for (auto& pt : route) {
					delete pt;				//careful with delete
				}
				route.clear();
			}
			routes.clear();

			// Copy new route vectors
			for (const auto& route : other.routes) {
				vector<Pt*> newRoute;
				for (const auto& pt : route) {
					newRoute.push_back(new Pt(*pt));
				}
				routes.push_back(newRoute);
			}
		}
		return *this;
	}

private:
	static int count;
};

struct FullSoln {
public:
	FullSoln(const MSSoln& msSoln, const vector<DroneSoln*>& droneSolns) :
		ID(count++), msSoln(msSoln), droneSolns() {
		// Create new DroneSoln objects with new memory locations for pointers
		for (auto* drone : droneSolns) {
			this->droneSolns.push_back(new DroneSoln(*drone));
		}
	}
	FullSoln(const MSSoln msSoln) :
		ID(count++), msSoln(msSoln)
	{}
	// Copy constructor for deep copy
	FullSoln(const FullSoln& other) :
		ID(count++), msSoln(other.msSoln), // Deep copy
		droneSolns(), sa_log(other.sa_log)
		{
		for (auto& dronesoln : other.droneSolns) {
			this->droneSolns.push_back(new DroneSoln(*dronesoln));
		}
	}
	// IN_SWAPS: Copy constructor with additional routes parameter
	FullSoln(const FullSoln& other, const vector<vector<Pt*>>& routes, int c) :
		ID(count++), msSoln(other.msSoln), // Deep copy
		droneSolns()
		//, greedy(other.greedy), without_clust(other.without_clust), within_clust(other.within_clust), greedy_again(other.greedy_again) 
		{
		// Copy new DroneSoln objects with updated routes
		for (int i = 0; i < other.droneSolns.size(); ++i) {
			// Ensure the routes vector is not out of bounds
			if (i == c) {
				// Modify the route for the specified index (c)
				DroneSoln* modifiedDroneSoln = new DroneSoln(*other.droneSolns[i]);
				modifiedDroneSoln->routes = routes;
				this->droneSolns.push_back(modifiedDroneSoln);
			}
			else {
				// Use the original route if no replacement is provided
				this->droneSolns.push_back(new DroneSoln(*other.droneSolns[i]));
			}
		}
	}
	
	//// NEW -- OUT_SWAPS: Copy constructor with additional routes parameters
	//FullSoln(const MSSoln& msSoln, const vector<DroneSoln*>& droneSolns) :
	//	ID(count++), msSoln(msSoln), droneSolns(droneSolns) {}

	const int ID;
	/*const*/ MSSoln msSoln;
	vector<DroneSoln*> droneSolns;
	//SAparams sa_params;
	SAlog sa_log;


	/// <summary>
	/// Method to get the total distance of the solution.
	/// Total dist = w_ms*(ms) + w_d*(sum(drone))
	/// </summary>
	/// <param name="print">= false</param>
	/// <returns></returns>
	double getTotalDist(bool print = false) const {
		double dist_ms = INST.weights.first * msSoln.getDist(print);
		double dist_drones = 0;
		if (print) printf("\nWEIGHTED MS Dist: \t%.2f\t\t(w_ms = %.1f)\n\tw_d = %.1f", dist_ms, INST.weights.first, INST.weights.second);
		for (auto& droneSoln : droneSolns) {
			double dist_drone = 0;
			if (print) printf("\nDroneSoln ID: \t%d\t\tDist:", droneSoln->ID);
			for (auto& route : droneSoln->routes) {
				double leg = INST.weights.second * droneSoln->getDroneRouteDist(route);
				if (print) printf("\t+%.2f", leg);
				dist_drone += leg;
			}
			if (print) printf("\t = %.2f", dist_drone);
			dist_drones += dist_drone;
		}
		double dist_total = dist_ms + dist_drones;
		if (print) {
			printf("\nDrone Dist: \t\t%.2f\n\nTotal Dist: \t%.2f\n", dist_drones, dist_total);
		}
		return dist_total;
	}

	// Copy assignment operator for deep copy
	FullSoln& operator=(const FullSoln& other) {
		if (this != &other) {
			// update msSoln
			msSoln = other.msSoln;

			// Release existing DroneSoln objects
			for (auto& ptr : droneSolns) {
				delete ptr;
			}
			droneSolns.clear();

			// Copy new DroneSoln objects
			for (auto& dronesoln : other.droneSolns) {
				droneSolns.push_back(new DroneSoln(*dronesoln/*, true*/)); // Pass 'true' to avoid copying launchPts
			}
			//// Update other members accordingly
			//msSoln = other.msSoln;		// Deep copy MSSoln
			sa_log = other.sa_log;
		}
		return *this;
	}

	// Method to set SAlog attributes
	void setSAlog(const vector<double>& sa_new,
		const vector<double>& sa_current,
		const vector<double>& sa_best,
		const vector<double>& sa_temp,
		const SAparams& params) {
		this->sa_log.best_dist = sa_best;
		this->sa_log.current_dist = sa_current;
		this->sa_log.new_dist = sa_new;
		this->sa_log.temp = sa_temp;
		this->sa_log.params = params;
	}

private:
	static int count;
};

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////