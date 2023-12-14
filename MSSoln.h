//#pragma once
//using namespace std;
//#include <iostream>
//#include <vector>
//#include <cmath>
//#include <utility>
//
//#include "class_def.h"
//
//struct MSSoln {
//public:
//	//MSSoln(const Problem& inst) : ID(count++), inst(inst) {}
//	MSSoln(const Problem* inst, const ClusterSoln* clustSoln) :
//		ID(count++), inst(inst), clustSoln(clustSoln), launchPts(clustSoln->clusters.size() + 1, nullptr) {}
//
//	//MSSoln(const Problem& inst, vector<Cluster*> clustOrder/*, vector<Route_MS*> routes*/) :
//	//	ID(count++), inst(inst), clustOrder(clustOrder)/*, routes(routes)*/{}
//
//	const int ID;
//	const Problem* inst;				// problem instance
//	const ClusterSoln* clustSoln;
//
//	vector<Cluster*> clustOrder;		// ordered clusters		////vector<int> clustOrder;		// ordered clusters by ID
//	vector<Pt*> launchPts;
//
//	//check this works correctly between correct points...
//	vector<vector<double>> ordered_dMatrix() {
//		vector<vector<double>> dMatrix;
//		vector<double> depotDists;
//		depotDists.push_back(0.0);
//		for (auto& clust : clustOrder) {
//			double dist = sqrt(pow(inst->ms.depot.x - clust->getCentroid().x, 2) + pow(inst->ms.depot.y - clust->getCentroid().y, 2));
//			depotDists.push_back(dist);
//		}
//		dMatrix.push_back(depotDists);
//		// is this dangerous in case where order changes?!
//
//		for (int c = 0; c < clustOrder.size(); c++) {
//			vector<double> clustDists;
//			clustDists.push_back(depotDists[c + 1]);
//			for (int d = 0; d < clustOrder.size(); d++) {
//				double dist = sqrt(pow(clustOrder[c]->getCentroid().x - clustOrder[d]->getCentroid().x, 2) + pow(clustOrder[c]->getCentroid().y - clustOrder[d]->getCentroid().y, 2));
//				clustDists.push_back(dist);
//			}
//			dMatrix.push_back(clustDists);
//		}
//		return dMatrix;
//	}
//
//	double getDist() {
//		double dist = //calculatePtDistance(inst->ms.depot, clustOrder[0]->getCentroid());
//			sqrt(pow(inst->ms.depot.x - launchPts[0]->x, 2) + pow(inst->ms.depot.y - launchPts[0]->y, 2));
//		for (int c = 0; c < clustOrder.size(); c++) {
//			dist += sqrt(pow(launchPts[c]->x - launchPts[c + 1]->x, 2) + pow(launchPts[c]->y - launchPts[c + 1]->y, 2));
//			//calculatePtDistance(
//			//	clustOrder[c]->getCentroid(), 
//			//	clustOrder[c+1]->getCentroid()
//			//);
//		}
//		dist += sqrt(pow(inst->ms.depot.x - launchPts[clustOrder.size()]->x, 2) + pow(inst->ms.depot.y - launchPts[clustOrder.size()]->y, 2));//calculatePtDistance(inst->ms.depot, clustOrder[clustOrder.size()-1]->getCentroid());
//		//sqrt(pow(inst->ms.depot.x - launchPts[clustOrder.size() - 1]->x, 2) + pow(inst->ms.depot.y - launchPts[clustOrder.size() - 1]->y, 2));
//		return dist;
//	}
//
//	vector<Pt*> getRoute() {			//update/check this! use mp's of centroids...
//		vector<Pt*> route;//(launchPts.size() + 1, nullptr);
//		route.push_back(new Pt(inst->ms.depot)); // Assuming Pt has a copy/move constructor
//		for (auto& pt : launchPts) {
//			route.push_back(pt);
//		}
//		route.push_back(new Pt(inst->ms.depot)); // Assuming Pt has a copy/move constructor
//		return route;
//	}
//	//Cluster& getCluster(int ID) {
//	//	for (auto& clust : clustOrder) {
//	//		if (clust->getID() == ID) {
//	//			return *clust;
//	//		}
//	//	}
//	//}
//
//	//int getClusterIndex(Cluster& cluster) {
//	//	return cluster.getID() - 1;
//	//}
//private:
//	static int count;
//};
//
