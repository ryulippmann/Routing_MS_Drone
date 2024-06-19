#pragma once
#include <random>

random_device rd;       // Seed for the random number generator
mt19937 gen(rd());      // Mersenne Twister engine

double sum(const vector<double>& vec_to_sum) {
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

double CalcCentroidDist(const vector<Pt>& cluster, const Pt& centroid) {
    double summing_centroid_dist = 0;
    for (int i = 0; i < cluster.size(); ++i) {  // i = node id in cluster
        summing_centroid_dist += calculatePtDistance(cluster[i], centroid);
    }
    return summing_centroid_dist;
}
double CalcCentroidDist(const vector<Pt*>& cluster, const Pt& centroid) {
    double summing_centroid_dist = 0;
    for (const auto& node : cluster) {
        summing_centroid_dist += calculatePtDistance(*node, centroid);
    }
    return summing_centroid_dist;
}

// Function to find index of Pt in vector by Pt ID
int findIndexByID(int targetID, const vector<Pt*>& myList, pair<Pt*, Pt*> launchPts) {
    for (int index = 0; index < myList.size(); index++) {
        if (myList[index]->ID == targetID) { return index+1; }
	}
    if (launchPts.first->ID == targetID) { return 0; }
    else if (launchPts.second->ID == targetID) { return myList.size() + 1; }
	else throw invalid_argument("Pt ID not found");
	//
	return -1;  // Return indicator that ID was not found
}
int findIndexByID(int targetID, const vector<Pt*>& myList) {
    for (int index = 0; index < myList.size(); index++) {
        if (myList[index]->ID == targetID) { return index; }
    }
    throw invalid_argument("Pt ID not found");
    return -1;  // Return indicator that ID was not found
}

/// <summary>
/// get random number between 0 and 'size-1'
/// </summary>
/// <param name="size"></param>
/// <returns></returns>
int getRandomNumber(int size/*, int randomSeed = -1*/) {
    mt19937 gen(rd());
    uniform_int_distribution<int> dist(0, size - 1);
    return dist(gen);
}
