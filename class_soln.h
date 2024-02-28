#pragma once

struct ClusterSoln {
public:
	ClusterSoln(const Problem& inst) : ID(count++)/*, inst(inst)*/ {}
	ClusterSoln(const Problem& inst, /*const*/ vector<Pt*> reefs) : ID(count++),/* inst(inst),*/ reefs(reefs) {}
	// Copy constructor for deep copy
	ClusterSoln(const ClusterSoln& other) :
		ID(other.ID), /*inst(other.inst), */reefs() {
		// Copy new Pt objects in the reefs vector
		for (auto& reef : other.reefs) {
			this->reefs.push_back(new Pt(*reef));
		}
	}

	const int ID;
	//const Problem& inst;
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
		ID(count++), /*inst(inst),*/ clusters(clustSolns), launchPts(clustSolns.size() + 1, nullptr) {}
	// Copy constructor for deep copy
	MSSoln(const MSSoln& other) :
		ID(count++), /*inst(other.inst),*/ clusters(), launchPts(other.launchPts) {
		// Copy new ClusterSoln objects
		for (auto& cluster : other.clusters) {
			this->clusters.push_back(new ClusterSoln(*cluster)); 
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
			double dist = sqrt(pow(inst.ms.depot.x - clust->getCentroid().x, 2) + pow(inst.ms.depot.y - clust->getCentroid().y, 2));
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
			double dist = sqrt(pow(inst.ms.depot.x - launch->x, 2) + pow(inst.ms.depot.y - launch->y, 2));
			depotDists.push_back(dist);
		}
		dMatrix.push_back(depotDists); // is this dangerous in case where order changes?!

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
	double getDist() const {
		if (launchPts.size() == 0) {
			throw runtime_error("Launch points not set!");
			return -1;
		}
		double dist = sqrt(pow(inst.ms.depot.x - launchPts[0]->x, 2) + pow(inst.ms.depot.y - launchPts[0]->y, 2));
		for (int c = 0; c < clusters.size(); c++) {
			dist += sqrt(pow(launchPts[c]->x - launchPts[c + 1]->x, 2) + pow(launchPts[c]->y - launchPts[c + 1]->y, 2));
		}
		dist += sqrt(pow(inst.ms.depot.x - launchPts[clusters.size()]->x, 2) + pow(inst.ms.depot.y - launchPts[clusters.size()]->y, 2));
		//sqrt(pow(inst.ms.depot.x - launchPts[clustOrder.size() - 1]->x, 2) + pow(inst.ms.depot.y - launchPts[clustOrder.size() - 1]->y, 2));
		return dist;
	}

	vector<Pt*> getRoute() {			//update/check this! use mp's of centroids...
		vector<Pt*> route;//(launchPts.size() + 1, nullptr);
		route.push_back(new Pt(inst.ms.depot)); // Assuming Pt has a copy/move constructor
		for (auto& pt : launchPts) {
			route.push_back(pt);
		}
		route.push_back(new Pt(inst.ms.depot)); // Assuming Pt has a copy/move constructor
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
			//// Deep copy launchPts
			//for (auto& pt : launchPts) { delete pt; }			//careful with delete
			//launchPts.clear();
			//for (auto& pt : other.launchPts) {
			//	launchPts.push_back(new Pt(*pt));
			//}
		}
		return *this;
	}

private:
	static int count;
};

struct TenderSoln {		//typedef vector<Pt*> Route_Tender;
public:
	TenderSoln(ClusterSoln cluster) : ID(count++), cluster(cluster) {}
	TenderSoln(ClusterSoln cluster, vector<vector<Pt*>> routes, pair<Pt*, Pt*> launchPts) :
		ID(count++), cluster(cluster), routes(routes), launchPts(launchPts) {}//, greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}

	// Copy constructor for deep copy
	/* Could include switch for deep or shallow copy
	// User-defined copy constructor with option for deep or shallow copy
	MyClass(const MyClass& other, bool deepCopy = true) {
		if (deepCopy) {
			data = new int(*other.data);
		}
		else {
			data = other.data;  // Shallow copy
		}
	}*/
	TenderSoln(const TenderSoln& other, bool reef_copy = false) :
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
	ClusterSoln/***/ cluster;
	vector<vector<Pt*>> routes;
	pair<Pt*, Pt*> launchPts;

	// does this include launchPts in route?! - check!
	// route dist for cluster c
	double getTenderRouteDist(int c = -1) const {
		vector<Pt*> route = routes[c];
		//route.insert(route.begin(), launchPts.first);	// Insert launchPts.first at beginning of route vector
		//route.push_back(launchPts.second);				// Push launchPts.second at end of route vector
		//printf("%d\t(%.2f, %.2f)\n", route[c]->ID, route[c]->x, route[c]->y);
		double route_dist = 0;
		vector<vector<double>> dMatrix = cluster.getdMatrix(launchPts);
		for (int i = 0; i < route.size() - 1; ++i) {
			route_dist += dMatrix[findIndexByID(route[i]->ID, route)][findIndexByID(route[i + 1]->ID, route)];
		}
		return route_dist;
	}
	// route dist for given route
	double getTenderRouteDist(vector<Pt*> route) const {
		//vector<Pt*> route = routes[c];
		double route_dist = 0;
		//vector<vector<double>> dMatrix = cluster->getdMatrix(launchPts);
		for (int i = 0; i < route.size() - 1; ++i) {
			route_dist += calculatePtDistance(route[i], route[i + 1]);
			//route_dist += dMatrix[findIndexByID(route[i]->ID, cluster->reefs, launchPts)][findIndexByID(route[i + 1]->ID, cluster->reefs, launchPts)];
		}
		return route_dist;
	}
	// total dist for ALL tenders
	double getTotalDist() {
		double dist = 0;
		for (int c = 0; c < routes.size(); c++) {
			dist += getTenderRouteDist(c);
		}
		return dist;
	}

	// Copy assignment operator for deep copy
	TenderSoln& operator=(const TenderSoln& other) {
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
	FullSoln(const MSSoln msSoln, vector<TenderSoln*>& tenderSolns) :
		ID(count++), msSoln(msSoln), tenderSolns(tenderSolns) //, greedy(true), without_clust(false), within_clust(false), greedy_again(false) 
		{}
	FullSoln(const MSSoln msSoln) :
		ID(count++), msSoln(msSoln) //, greedy(true), without_clust(false), within_clust(false), greedy_again(false) 
	{}
	// Copy constructor for deep copy
	FullSoln(const FullSoln& other) :
		ID(count++), msSoln(other.msSoln), // Deep copy
		tenderSolns()/*, greedy(other.greedy), without_clust(other.without_clust), within_clust(other.within_clust), greedy_again(other.greedy_again)*/ 
		{
		for (auto& tendersoln : other.tenderSolns) {
			this->tenderSolns.push_back(new TenderSoln(*tendersoln));
		}
	}
	// IN_SWAPS: Copy constructor with additional routes parameter
	FullSoln(const FullSoln& other, const vector<vector<Pt*>>& routes, int c) :
		ID(count++), msSoln(other.msSoln), // Deep copy
		tenderSolns()
		//, greedy(other.greedy), without_clust(other.without_clust), within_clust(other.within_clust), greedy_again(other.greedy_again) 
		{
		// Copy new TenderSoln objects with updated routes
		for (int i = 0; i < other.tenderSolns.size(); ++i) {
			// Ensure the routes vector is not out of bounds
			if (i == c) {
				// Modify the route for the specified index (c)
				TenderSoln* modifiedTenderSoln = new TenderSoln(*other.tenderSolns[i]);
				modifiedTenderSoln->routes = routes;
				this->tenderSolns.push_back(modifiedTenderSoln);
			}
			else {
				// Use the original route if no replacement is provided
				this->tenderSolns.push_back(new TenderSoln(*other.tenderSolns[i]));
			}
		}
	}
	// OUT_SWAPS: Copy constructor with additional routes parameters
	FullSoln(const FullSoln& other, const pair <vector<vector<Pt*>>, vector<vector<Pt*>>>& routes, pair<ClusterSoln, ClusterSoln> clusters, pair<int, int> c) :
		ID(count++), msSoln(other.msSoln), // Deep copy - UPDATE based on TenderSoln!
		tenderSolns()
		//, greedy(other.greedy), without_clust(other.without_clust), within_clust(other.within_clust), greedy_again(other.greedy_again) 
		{

		// Copy new TenderSoln objects with updated routes
		for (int i = 0; i < other.tenderSolns.size(); ++i) {
			if (i == c.first) {
				// Modify the route for the specified index (c)
				TenderSoln modifiedTenderSoln = TenderSoln(*other.tenderSolns[i], true);
				modifiedTenderSoln.routes = routes.first;
				//create new clusterSoln* with updated reefs
				modifiedTenderSoln.cluster = clusters.first;
				this->tenderSolns.push_back(new TenderSoln(modifiedTenderSoln));
			}
			else if (i == c.second) {
				// Modify the route for the specified index (c)
				TenderSoln* modifiedTenderSoln = new TenderSoln(*other.tenderSolns[i]);
				modifiedTenderSoln->routes = routes.second;
				//create new clusterSoln* with updated reefs
				modifiedTenderSoln->cluster = clusters.second;
				this->tenderSolns.push_back(modifiedTenderSoln);
			}
			else {
				// Use the original route if no replacement is provided
				this->tenderSolns.push_back(new TenderSoln(*other.tenderSolns[i]));
			}
		}
	}

	const int ID;
	/*const*/ MSSoln msSoln;
	vector<TenderSoln*> tenderSolns;
	//bool greedy;
	//bool without_clust;
	//bool within_clust;
	//bool greedy_again;

	// Total dist = ms + sum(tender)
	double getTotalDist() const {
		double dist = msSoln.getDist();
		for (auto& tenderSoln : tenderSolns) {
			for (auto& route : tenderSoln->routes) {
				dist += tenderSoln->getTenderRouteDist(route);
			}
		}
		return dist;
	}

	// Copy assignment operator for deep copy
	FullSoln& operator=(const FullSoln& other) {
		if (this != &other) {
			// Release existing TenderSoln objects
			for (auto& ptr : tenderSolns) {
				delete ptr;
			}
			tenderSolns.clear();

			// Copy new TenderSoln objects
			for (auto& tendersoln : other.tenderSolns) {
				tenderSolns.push_back(new TenderSoln(*tendersoln/*, true*/)); // Pass 'true' to avoid copying launchPts
			}
			//// Update other members accordingly
			//msSoln = other.msSoln;		// Deep copy MSSoln
			//greedy = other.greedy;
			//without_clust = other.without_clust;
			//within_clust = other.within_clust;
			//greedy_again = other.greedy_again;
		}
		return *this;
	}

private:
	static int count;
};

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
