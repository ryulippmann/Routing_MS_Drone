#pragma once
#include <vector>
//#include <iostream>
#include <cmath>
using namespace std;

//#ifndef class_def_h
//#define class_def_h
//
//struct Pt;
//
//#endif // !class_def_h


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

double calcRouteDist(vector<vector<Pt*>> routes) {
    // this doesn't account for free link from last node in route to launchpt
	double summing_route_dist = 0;
    for (const auto& route : routes) {
        for (int i = 0; i < route.size() - 1; ++i) {
			summing_route_dist += calculatePtDistance(*route[i], *route[i + 1]);
		}
	}
	return summing_route_dist;
}

// Function to find index of Pt in vector by Pt ID
int findIndexByID(int targetID, const vector<Pt*>& myList) {
    for (int index = 0; index < myList.size(); index++) {
        if (myList[index]->ID == targetID) {
			return index+1;
		}
	}
	return -1;  // Return a special value (e.g., -1) to indicate that the ID was not found
}

// Function to get a reference to Pt based on ID
Pt* getPtByID(int targetID, vector<Pt*> route) {
    for (auto& Pt : route) {
        if (Pt->ID == targetID) {
            return Pt;
        }
    }
    return nullptr; // ID not found
}
//double getRouteDist(vector<Pt*> route, vector<vector<double>> dMatrix, Cluster* cluster) {
//    double summing_route_dist = 0;
//    for (int i = 0; i < route.size() - 1; ++i) {
//        summing_route_dist += dMatrix[findIndexByID(route[i]->ID, cluster->reefs)][findIndexByID(route[i + 1]->ID, cluster->reefs)];
//    }
//    return summing_route_dist;
//}
