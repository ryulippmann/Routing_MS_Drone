#pragma once
#include <random>

random_device rd;       // Seed for the random number generator
mt19937 gen(rd());      // Mersenne Twister engine

double sum(vector<double>& vec_to_sum) {
    double sum_of_elems = 0.0;
    for (const auto& n : vec_to_sum) {
        sum_of_elems += n;
    }
    return sum_of_elems;
}//sum

Pt calcCentroid(const vector<Pt> cluster) {
    double sumX = 0.0, sumY = 0.0;
    for (const Pt& p : cluster) {
        sumX += p.x;
        sumY += p.y;
    }
    if (!cluster.empty()) {
        return Pt(sumX / cluster.size(), sumY / cluster.size());
    }
    else { throw invalid_argument("Cluster empty!"); }
}
Pt calcCentroid(const vector<Pt*> cluster) {
    double sumX = 0.0, sumY = 0.0;
    for (const Pt* p : cluster) {
        sumX += p->x;
        sumY += p->y;
    }
    if (!cluster.empty()) {
        return Pt(sumX / cluster.size(), sumY / cluster.size());
    }
    else { throw invalid_argument("Cluster empty!"); }
}

double calculatePtDistance(const Pt& a, const Pt& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}//calculatePtDistance
double calculatePtDistance(const Pt& a, const Pt* b) {
    return sqrt(pow(a.x - b->x, 2) + pow(a.y - b->y, 2));
}//calculatePtDistance
double calculatePtDistance(const Pt* a, const Pt* b) {
    return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2));
}//calculatePtDistance

double CalcCentroidDist(const vector<Pt> cluster, const Pt centroid) {
    double summing_centroid_dist = 0;
    for (int i = 0; i < cluster.size(); ++i) {  // i = node id in cluster
        summing_centroid_dist += calculatePtDistance(cluster[i], centroid);
    }
    return summing_centroid_dist;
}
double CalcCentroidDist(const vector<Pt*> cluster, const Pt& centroid) {
    double summing_centroid_dist = 0;
    for (const auto& node : cluster) {
        summing_centroid_dist += calculatePtDistance(*node, centroid);
    }
    return summing_centroid_dist;
}

// Function to find index of Pt in vector by Pt ID
int findIndexByID(int targetID, const vector<Pt*>& myList, pair<Pt*, Pt*> launchPts = make_pair(nullptr, nullptr)) {
    for (int index = 0; index < myList.size(); index++) {
        if (myList[index]->ID == targetID) { return index+1; }
	}
    if (launchPts.first->ID == targetID) { return 0; }
    else if (launchPts.second->ID == targetID) { return myList.size() + 1; }
	else throw invalid_argument("Pt ID not found");
	//
	return -1;  // Return a special value (e.g., -1) to indicate that the ID was not found
}

int getRandomNumber(int size/*, int randomSeed = -1*/) {
    mt19937 gen(rd());
    uniform_int_distribution<int> dist(0, size - 1);
    return dist(gen);
}

//vector<vector<double>> centroidMatrix(Problem& inst, vector<Pt*> reefs) {
//    vector<vector<double>> centroidMatrix;
//    //centroids.insert(centroids.begin(), ClusterPoint(depot.first, depot.second));
//    vector<double> depot_row;
//    depot_row.push_back(0);
//    for (int i = 0; i < reefs.size(); i++) {		// for each cluster
//        depot_row.push_back(calculatePtDistance(inst.ms.depot, reefs[i]));
//    }
//    centroidMatrix.push_back(depot_row);
//
//    for (int i = 0; i < reefs.size(); i++) {		// for each cluster
//        vector<double> row(reefs.size() + 1);
//        row[0] = depot_row[i + 1];
//        for (int j = 0; j < reefs.size(); j++) {	// for each cluster
//            row[j + 1] = calculatePtDistance(reefs[i], reefs[j]);
//        }
//        centroidMatrix.push_back(row);
//    }
//    return centroidMatrix;
//}

//double calcRouteDist(vector<vector<Pt*>> routes) {
//    // this doesn't account for free link from last node in route to launchpt
//	double summing_route_dist = 0;
//    for (const auto& route : routes) {
//        for (int i = 0; i < route.size() - 1; ++i) {
//			summing_route_dist += calculatePtDistance(*route[i], *route[i + 1]);
//		}
//	}
//	return summing_route_dist;
//}

//// Function to get a reference to Pt based on ID
//Pt* getPtByID(int targetID, vector<Pt*> route) {
//    for (auto& Pt : route) {
//        if (Pt->ID == targetID) {
//            return Pt;
//        }
//    }
//    if (targetID == 0)  return route[0];
//	else if (targetID == route.size() - 1) return route.back();
//	else throw invalid_argument("Pt ID not found");
//    //return nullptr; // ID not found
//}

//double getRouteDist(vector<Pt*> route, vector<vector<double>> dMatrix, Cluster* cluster) {
//    double summing_route_dist = 0;
//    for (int i = 0; i < route.size() - 1; ++i) {
//        summing_route_dist += dMatrix[findIndexByID(route[i]->ID, cluster->reefs)][findIndexByID(route[i + 1]->ID, cluster->reefs)];
//    }
//    return summing_route_dist;
//}

//// return string "in" or "out" based on bool value
//string boolToString(bool value) {
//    return value ? "in" : "out";
//}
