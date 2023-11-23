#pragma once
//#include <vector>
//#include <iostream>
#include <cmath>
using namespace std;

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

