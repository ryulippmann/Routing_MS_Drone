#pragma once

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
		Pt centroid(x / reefs.size(), y / reefs.size());
		return centroid;
	}

	// includes launchpts and free link back to launchpt
	vector<vector<double>> getdMatrix(const int c, pair <Pt*, Pt*> launchpts) {	//mothership, cluster
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
			clustDists.push_back(depotDists[c + 1]);
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
	TenderSoln(Cluster* cluster, vector<vector<Pt*>>& routes, pair<Pt*, Pt*> launchPts) :
		ID(count++), cluster(cluster), routes(routes), launchPts(launchPts) {}//, greedy(true), without_clust(false), within_clust(false), greedy_again(false) {}
	//TenderSoln(Cluster& cluster, vector<Route_Tender> routes, bool greedy, bool without_clust, bool within_clust, bool greedy_again) :
	//	ID(count++), cluster(cluster), routes(routes), greedy(greedy), without_clust(without_clust), within_clust(within_clust), greedy_again(greedy_again) {}

	const int ID;

	Cluster* cluster;
	vector<Route_Tender> routes;
	pair<Pt*, Pt*> launchPts;

	vector<double> getTenderRouteDists(int c) {	//Cluster* cluster, Route_Tender route, pair<Pt*, Pt*> launchPts, vector<vector<double>> dMatrix
		vector<double> route_dist;
		for (int i = 0; i < routes.size(); i++) {
			vector<Pt*> route = routes[i];
			route.insert(route.begin(), launchPts.first);	// Insert launchPts.first at beginning of route vector
			route.push_back(launchPts.second);				// Push launchPts.second at end of route vector
			printf("%d\t(%.2f, %.2f)\n", route[i]->ID, route[i]->x, route[i]->y);

			double summing_route_dist = 0;
			vector<vector<double>> dMatrix = cluster->getdMatrix(c, launchPts);
			for (int i = 0; i < route.size() - 1; ++i) {
				summing_route_dist += dMatrix[findIndexByID(route[i]->ID, cluster->reefs)][findIndexByID(route[i + 1]->ID, cluster->reefs)];
			}
			route_dist.push_back(summing_route_dist);//getRouteDist(route, cluster->getdMatrix(c, launchPts), cluster));
		}

		//double summing_route_dist = 0;
		//for (int i = 0; i < route.size() - 1; ++i) {
		//	summing_route_dist += dMatrix[findIndexByID(route[i]->ID, cluster->reefs, launchPts)][findIndexByID(route[i+1]->ID, cluster->reefs, launchPts)];
		//}
		return route_dist;
	}

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
