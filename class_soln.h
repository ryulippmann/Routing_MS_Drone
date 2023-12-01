#pragma once

////////////////////////////////////////////////////////////////////////////

//struct Cluster {
//public:
//	Cluster(vector<Pt*> reefs/*, Pt start, Pt end*/) : ID(count++), reefs(reefs)/*, start(start), end(end)*/ {}
//
//	const int ID;
//	const vector<Pt*> reefs;
//	//Pt centroid = this->getCentroid(/*reefs*/);
//	Pt getCentroid() {
//		double x = 0.0;
//		double y = 0.0;
//		for (auto& reef : reefs) {
//			x += reef->x;
//			y += reef->y;
//		}
//		Pt centroid(x / reefs.size(), y / reefs.size());
//		return centroid;
//	}
//
//	// includes launchpts and free link back to launchpt
//	vector<vector<double>> getdMatrix(const int c, pair <Pt*, Pt*> launchpts) {	//mothership, cluster
//		vector<double> launchDists{ 0.0 };			// launchpt to itself = 0
//		for (auto& reef : reefs) {					// launchpt to each reef
//			double dist = sqrt(pow(reef->x - launchpts.first->x, 2) + pow(reef->y - launchpts.first->y, 2));
//			launchDists.push_back(dist);			// dist launchpt to each reef
//		} launchDists.push_back(DBL_MAX);			// launchpt to retrieve pt = inf
//
//		vector<vector<double>> dMatrix;
//		dMatrix.push_back(launchDists);				// launchpt to each reef
//
//		vector<double> retrieveDists;				// retrieve pt to each reef
//		retrieveDists.push_back(0.0);				// retrieve pt to itself = 0
//		for (auto& reef : reefs) {					// retrieve pt to each reef
//			double dist = sqrt(pow(reef->x - launchpts.second->x, 2) + pow(reef->y - launchpts.second->y, 2));
//			retrieveDists.push_back(dist);			// dist retrieve pt to each reef
//		} retrieveDists.push_back(0.0);				// FREE-LINK: retrieve pt return to launchpt
//
//		for (int r = 0; r < reefs.size(); r++) {
//			vector<double> row;
//			row.push_back(launchDists[r + 1]);
//
//			for (int s = 0; s < reefs.size(); s++) {
//				double dist = sqrt(pow(reefs[r]->x - reefs[s]->x, 2) + pow(reefs[r]->y - reefs[s]->y, 2));
//				row.push_back(dist);
//			}
//			row.push_back(retrieveDists[r + 1]);
//			dMatrix.push_back(row);
//		}
//		dMatrix.push_back(retrieveDists);
//
//
//		return dMatrix;
//	}
//
//private:
//	static int count;
//};

////////////////////////////////////////////////////////////////////////////

struct ClusterSoln {
public:
	ClusterSoln(const Problem& inst) : ID(count++), inst(inst) {}
	ClusterSoln(const Problem& inst, const vector<Pt*> reefs) : ID(count++), inst(inst), reefs(reefs) {}
	//ClusterSoln(const vector<Pt*> reefs, const vector<vector<double>>& centroidMatrix) :
	//	ID(count++), reefs(reefs), centroidMatrix(centroidMatrix) {}
	//ClusterSoln(const vector<Cluster*>& clusters) : ID(count++), clusters(clusters) {}
	//ClusterSoln(const vector<Cluster*>& clusters, const vector<vector<double>>& centroidMatrix) :
	//	ID(count++), clusters(clusters), centroidMatrix(centroidMatrix) {}

	const int ID;
	const Problem& inst;
	const vector<Pt*> reefs;

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

	//vector<vector<double>> calc_centMatrix(const vector<ClusterSoln*>& clusters, const Pt depot) {
	//	vector<vector<double>> centroidMatrix;
	//	//centroids.insert(centroids.begin(), ClusterPoint(depot.first, depot.second));
	//	vector<double> depot_row;
	//	depot_row.push_back(0);
	//	for (int i = 0; i < clusters.size(); i++) {		// for each cluster
	//		depot_row.push_back(calculatePtDistance(depot, clusters[i]->getCentroid()));
	//	}
	//	centroidMatrix.push_back(depot_row);
	//
	//	for (int i = 0; i < clusters.size(); i++) {		// for each cluster
	//		vector<double> row(clusters.size() + 1);
	//		row[0] = depot_row[i + 1];
	//		for (int j = 0; j < clusters.size(); j++) {	// for each cluster
	//			row[j + 1] = calculatePtDistance(clusters[i]->getCentroid(), clusters[j]->getCentroid());
	//		}
	//		centroidMatrix.push_back(row);
	//	}
	//
	//	return centroidMatrix;
	//}

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
	vector<vector<double>> getdMatrix(const int c, pair <Pt*, Pt*> launchpts) const {	//mothership, cluster
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

private:
	static int count;
};

struct MSSoln {
public:
	//MSSoln(const Problem& inst) : ID(count++), inst(inst) {}
	MSSoln(const Problem& inst, vector<ClusterSoln*> clustSolns) :
		ID(count++), inst(inst), clusters(clustSolns), launchPts(clustSolns.size() + 1, nullptr) {}
	//MSSoln(const Problem& inst, vector<Cluster*> clustOrder/*, vector<Route_MS*> routes*/) :
	//	ID(count++), inst(inst), clustOrder(clustOrder)/*, routes(routes)*/{}

	const int ID;
	const Problem& inst;				// problem instance
	vector<ClusterSoln*> clusters;

	//vector<Cluster*> clustOrder;		// ordered clusters		////vector<int> clustOrder;		// ordered clusters by ID
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
		double dist = //calculatePtDistance(inst.ms.depot, clustOrder[0]->getCentroid());
			sqrt(pow(inst.ms.depot.x - launchPts[0]->x, 2) + pow(inst.ms.depot.y - launchPts[0]->y, 2));
		for (int c = 0; c < clusters.size(); c++) {
			dist += sqrt(pow(launchPts[c]->x - launchPts[c + 1]->x, 2) + pow(launchPts[c]->y - launchPts[c + 1]->y, 2));
			//calculatePtDistance(
			//	clustOrder[c]->getCentroid(), 
			//	clustOrder[c+1]->getCentroid()
			//);
		}
		dist += sqrt(pow(inst.ms.depot.x - launchPts[clusters.size()]->x, 2) + pow(inst.ms.depot.y - launchPts[clusters.size()]->y, 2));//calculatePtDistance(inst.ms.depot, clustOrder[clustOrder.size()-1]->getCentroid());
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
	//typedef vector<Pt*> Route_Tender;
public:
	//TenderSoln() : ID(count++), cluster(cluster), greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	TenderSoln(ClusterSoln* cluster) : ID(count++), cluster(cluster) {}
	//TenderSoln(ClusterSoln* cluster, vector<Route_Tender>& routes) :
	//	ID(count++), cluster(cluster), routes(routes) {}//, greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	//TenderSoln(ClusterSoln* cluster, pair<Pt*, Pt*> launchPts) :
	//	ID(count++), cluster(cluster), launchPts(launchPts) {}//, greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	TenderSoln(ClusterSoln* cluster, vector<vector<Pt*>> routes, pair<Pt*, Pt*> launchPts) :
		ID(count++), cluster(cluster), routes(routes), launchPts(launchPts) {}//, greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	//TenderSoln(Cluster& cluster, vector<Route_Tender> routes, bool greedy, bool without_clust, bool within_clust, bool greedy_again) :
	//	ID(count++), cluster(cluster), routes(routes), greedy(greedy), without_clust(without_clust), within_clust(within_clust), greedy_again(greedy_again) {}

	const int ID;

	ClusterSoln* cluster;
	vector<vector<Pt*>> routes;
	pair<Pt*, Pt*> launchPts;

	double getTenderRouteDist(int c = -1) const {
		vector<Pt*> route = routes[c];
		//route.insert(route.begin(), launchPts.first);	// Insert launchPts.first at beginning of route vector
		//route.push_back(launchPts.second);				// Push launchPts.second at end of route vector
		//printf("%d\t(%.2f, %.2f)\n", route[c]->ID, route[c]->x, route[c]->y);

		double route_dist = 0;
		vector<vector<double>> dMatrix = cluster->getdMatrix(c, launchPts);
		//vector<Pt*> pt_list = cluster->reefs;
		//pt_list.insert(pt_list.begin(), launchPts.first);
		//pt_list.push_back(launchPts.second);
		for (int i = 0; i < route.size() - 1; ++i) {
			route_dist += dMatrix[findIndexByID(route[i]->ID, route)][findIndexByID(route[i + 1]->ID, route)];
		}
		//route_dist = summing_route_dist;//getRouteDist(route, cluster->getdMatrix(c, launchPts), cluster));
		return route_dist;
	}

	// total dist for all tenders
	double getTotalDist() {
		double dist = 0;
		for (int c = 0; c < routes.size(); c++) {
			dist += getTenderRouteDist(c);
		}
		return dist;
	}

	//// dist for each tender route in cluster c
	//vector<double> getTenderRouteDists(int c = 0) {		//Cluster* cluster, Route_Tender route, pair<Pt*, Pt*> launchPts, vector<vector<double>> dMatrix
	//	vector<double> route_dist;
	//	//if (c == 0) {						// if c == 0, then return route dists for all clusters
	//	for (int i = 0; i < routes.size(); i++) {
	//		vector<Pt*> route = routes[i];
	//		route.insert(route.begin(), launchPts.first);	// Insert launchPts.first at beginning of route vector
	//		route.push_back(launchPts.second);				// Push launchPts.second at end of route vector
	//		printf("%d\t(%.2f, %.2f)\n", route[i]->ID, route[i]->x, route[i]->y);
	//
	//		double summing_route_dist = 0;
	//		vector<vector<double>> dMatrix = cluster->getdMatrix(c, launchPts);
	//		for (int i = 0; i < route.size() - 1; ++i) {
	//			summing_route_dist += dMatrix[findIndexByID(route[i]->ID, cluster->reefs)][findIndexByID(route[i + 1]->ID, cluster->reefs)];
	//		}
	//		route_dist.push_back(summing_route_dist);//getRouteDist(route, cluster->getdMatrix(c, launchPts), cluster));
	//	}
	//	//}
	//	//else {								// else return route dist for cluster c
	//	//	vector<Pt*> route = routes[c];
	//	//	route.insert(route.begin(), launchPts.first);	// Insert launchPts.first at beginning of route vector
	//	//	route.push_back(launchPts.second);				// Push launchPts.second at end of route vector
	//	//	printf("%d\t(%.2f, %.2f)\n", route[c]->ID, route[c]->x, route[c]->y);
	//
	//	//	double summing_route_dist = 0;
	//	//	vector<vector<double>> dMatrix = cluster->getdMatrix(c, launchPts);
	//	//	for (int i = 0; i < route.size() - 1; ++i) {
	//	//		summing_route_dist += dMatrix[findIndexByID(route[i]->ID, cluster->reefs)][findIndexByID(route[i + 1]->ID, cluster->reefs)];
	//	//	}
	//	//	route_dist.push_back(summing_route_dist);//getRouteDist(route, cluster->getdMatrix(c, launchPts), cluster));
	//	//}
	//
	//	return route_dist;
	//}
private:
	static int count;
};

struct FullSoln {
public:
	//FullSoln(Problem* dat, MSSoln* msSoln, TenderSoln* tenderSoln, vector<Cluster*> clusters) :
	//		ID(count++), dat(dat), msSoln(msSoln), tenderSoln(tenderSoln), clusters(clusters) {}
	FullSoln(const MSSoln& msSoln, vector<TenderSoln>& tenderSolns) :
		ID(count++), msSoln(msSoln), tenderSolns(tenderSolns),
		greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	FullSoln(const MSSoln& msSoln) :
		ID(count++), msSoln(msSoln),
		greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	//FullSoln(const MSSoln& msSoln, vector< TenderSoln*> tenderSolns, bool greedy, bool without_clust, bool within_clust, bool greedy_again) :
	//	ID(count++), msSoln(msSoln), tenderSolns(tenderSolns), 
	//	greedy(greedy), without_clust(without_clust), within_clust(within_clust), greedy_again(greedy_again) {}
		
	// Copy constructor
	FullSoln(const FullSoln& other) :
		ID(other.ID), msSoln(other.msSoln), tenderSolns(other.tenderSolns), 
		greedy(other.greedy), without_clust(other.without_clust), within_clust(other.within_clust), greedy_again(other.greedy_again) {
	
	}

	const int ID;
	const MSSoln& msSoln;
	const vector<TenderSoln> tenderSolns;
	//const Problem* dat;			//contained in msSoln
	//ClusterSoln* clusterSolns;	//contained in msSoln
	//vector<Cluster*> clusters;	//contained in msSoln

	const bool greedy;
	const bool without_clust;
	const bool within_clust;
	const bool greedy_again;

	double getTotalDist() {
		double dist = msSoln.getDist();
		for (auto& tenderSoln : tenderSolns) {
			dist += tenderSoln.getTenderRouteDist();
		}
		return dist;
	}

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
