#pragma once
#include "class_def.h"
#include "calcs.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <functional>

Pt calcCentroid(const vector<ReefPt> cluster) {
    double sumX = 0.0, sumY = 0.0;
    for (const ReefPt& p : cluster) {
        sumX += p.x;
        sumY += p.y;
    }
    if (!cluster.empty()) {
        return Pt(sumX / cluster.size(), sumY / cluster.size());
    } else { throw invalid_argument("Cluster empty!"); }
}
Pt calcCentroid(const vector<ReefPt*> cluster) {
    double sumX = 0.0, sumY = 0.0;
    for (const ReefPt* p : cluster) {
        sumX += p->x;
        sumY += p->y;
    }
    if (!cluster.empty()) {
        return Pt(sumX / cluster.size(), sumY / cluster.size());
    } else { throw invalid_argument("Cluster empty!"); }
}

double reef_Pt_Dist(const ReefPt& a, const Pt& b) {
    double dx = a.getXY().first - b.getXY().first;
    double dy = a.getXY().second - b.getXY().second;
    return sqrt(dx * dx + dy * dy);
}

double centroid_dist(const vector<ReefPt>& cluster, const Pt& centroid) {
    double summing_centroid_dist = 0;
    for (size_t i = 0; i < cluster.size(); ++i) {
        summing_centroid_dist += reef_Pt_Dist(static_cast<ReefPt>(cluster[i]), centroid);
    }
    return summing_centroid_dist;
}

double CalcCentroidDist(const vector<ReefPt> cluster, const Pt centroid) {
    double summing_centroid_dist = 0;
    for (int i = 0; i < cluster.size(); ++i) {  // i = node id in cluster
        summing_centroid_dist += reef_Pt_Dist(cluster[i], centroid);
    }
    //centroid_dist[c] = ;
    return summing_centroid_dist;
}
double CalcCentroidDist(const vector<ReefPt*> cluster, const Pt& centroid) {
    double summing_centroid_dist = 0;
    for (const auto& node : cluster) {
        summing_centroid_dist += reef_Pt_Dist(*node, centroid);
    }
    return summing_centroid_dist;
}

pair< vector<ReefPt>, vector<ReefPt>> swapClusters(const vector<ReefPt> clust_a, const vector<ReefPt> clust_b, const int idx1, const int idx2) {
    vector<ReefPt> new_a;//(clust_a.size());
    vector<ReefPt> new_b;//(clust_b.size());
    //ReefPt temp = ReefPt(clust_a[idx1].getXY());        // create temp ReefPt to store clust_a[idx1]
    //ReefPt temp2 = clust_b[idx2];       // create temp ReefPt to store clust_b[idx2]
    //clust_a[idx1] = temp2;              //clust_b[idx2];      // swap clust_a[idx1] with clust_b[idx2]
    //clust_b[idx2] = ReefPt(clust_a[idx1].getXY());               // swap clust_b[idx2] with temp
    
    for (int i = 0; i < clust_a.size(); ++i) { 
        if (i != idx1) {
            new_a.push_back(ReefPt(clust_a[i]));
        } else { new_a.push_back(ReefPt(clust_b[idx2])); }
    }
    for (int i = 0; i < clust_b.size(); ++i) { 
        if (i != idx2) {
			new_b.push_back(clust_b[i]);
		} else { new_b.push_back(clust_a[idx1]); }
	}
    return make_pair(new_a, new_b);
}

void /*pair< vector<ReefPt>, vector<ReefPt>>*/ randomSwapClusters(vector<ReefPt*>& clust_a, vector<ReefPt*>& clust_b, const pair<double, double> dists, pair<Pt, Pt> centroids, const int randomSeed) {
    mt19937 gen(randomSeed);        //random_device rd; //mt19937 gen(rd());
    uniform_int_distribution<int> dist(0, min(clust_a.size(), clust_b.size()) - 1);     //!constrict swaps between launch/retrieve nodes[1,9] (for now)
    bool swap_print = false;
    //int list1 = dist(gen);
    //int list2 = dist(gen);
    int index1 = dist(gen);
    int index2 = dist(gen);

    if (swap_print) {
        printf("\nSwap element %d in cluster a", /*list1,*/ index1);
        for (auto& node : clust_a) { printf("\n\t%d:\t\t(%.1f, %.1f)", node->ID, node->x, node->y); }
        printf("\nwith element %d in cluster b\t", /*list2,*/ index2);
        for (auto& node : clust_b) { printf("\n\t%d:\t\t(%.1f, %.1f)", node->ID, node->x, node->y); }
    }
    //pair<vector<ReefPt>, vector<ReefPt>> new_clusts = swapClusters(clust_a, clust_b, index1, index2);     // swap nodes between clusters
    swap(clust_a[index1], clust_b[index2]);     // swap nodes between clusters

    vector<double> centroid_dist(2);        // 0 = clust_a, 1 = clust_b
    centroid_dist[0] = CalcCentroidDist(clust_a/*new_clusts.first*/, centroids.first);
    centroid_dist[1] = CalcCentroidDist(clust_b/*new_clusts.second*/, centroids.second);

    if (sum(centroid_dist) > dists.first + dists.second) {
        swap(clust_a[index1], clust_b[index2]);     // if condition met, swap BACK!
    } else if (swap_print) {
        //clust_a = new_clusts.first;
        //clust_b = new_clusts.second;
        //swapClusters(clust_a, clust_b, index1, index2);     // if condition not met, swap BACK!
        /*if (swap_print) {*/
            printf("\nNow element %d in cluster a", /*list1,*/ index1);
            for (const auto& node : clust_a) { printf("\n\t%d:\t\t(%.1f, %.1f)", node->ID, node->x, node->y); }
            printf("\nis element %d in cluster b\t", /*list2,*/ index2);
            for (const auto& node : clust_b) { printf("\n\t%d:\t\t(%.1f, %.1f)", node->ID, node->x, node->y); }
        //} //cout << "\n";
        //return make_pair(new_clusts.first, new_clusts.second);
    }
    // 
    //pair<vector<Reef_pt>, vector<Reef_pt>> new_clusts = make_pair(clust_a, clust_b);
    //return /*lists*/; //new_clusts
    return;// make_pair(clust_a, clust_b);
}

vector<Cluster*> kMeansConstrained(vector<ReefPt*> points, const int numClusters,
    int maxIterations = pow(10, 2),
    bool clusterPrint = false, bool csvPrint = false, int randomSeed = 12345) { //10^6 iter ~ 8-9 sec. 10^7 iter ~ 3 mins
    if (clusterPrint) cout << "\n---- kMEANS CONSTRAINED ----\n";
    int numPoints = points.size();
    // Check if desired cluster size is feasible
    if (numPoints % numClusters != 0) { throw invalid_argument("Clusters don't divide evenly..."); }
    int cluster_size = numPoints / numClusters;
    
    vector<vector<ReefPt*>> clusters(numClusters);
    vector<Pt*> centroids(numClusters);     // initialised empty. Populate with ClusterPoints             //= getClusterCentroids(clusters, true);
    vector<double> centroid_dist(numClusters);

    for (int c = 0; c < numClusters; c++) { // populate clusters with vector by cluster of vector of Reef Pts in cluster
        for (int i = 0; i < cluster_size; i++) {    // i = node id in cluster
            clusters[c].push_back(points[c * cluster_size + i]); // c*cluster_size = first node id in cluster
        }
        centroids[c] = new Pt(calcCentroid(clusters[c]));
        if (clusterPrint) printf("Centroid: %d\t(%.1f, %.1f)\n", c, centroids[c]->x, centroids[c]->y);
        centroid_dist[c] = CalcCentroidDist(clusters[c], *centroids[c]);
    }

    mt19937 gen(randomSeed);    //random_device rd; //mt19937 gen(rd());
    uniform_int_distribution<int> dist(0, clusters.size() - 1);     //!constrict swaps between launch/retrieve nodes[1,9] (for now)

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        int idx_clust_a = dist(gen);
        int idx_clust_b = dist(gen);
        if (idx_clust_a == idx_clust_b) { continue; } 
        if (clusterPrint) printf("\nswap clusters %d -> %d", idx_clust_a, idx_clust_b);
        /*pair<vector<ReefPt>, vector<ReefPt>> new_clusts = */
        randomSwapClusters(clusters[idx_clust_a], clusters[idx_clust_b], make_pair(centroid_dist[idx_clust_a], centroid_dist[idx_clust_b]), make_pair(*centroids[idx_clust_a], *centroids[idx_clust_b]), iteration);

        //clusters[idx_clust_a] = new_clusts.first;
        //clusters[idx_clust_b] = new_clusts.second;
        //UPDATE CENTROIDS!!
        centroids[idx_clust_a] = new Pt(calcCentroid(clusters[idx_clust_a]));
        centroids[idx_clust_b] = new Pt(calcCentroid(clusters[idx_clust_b]));
        //UPDATE TOTAL CENTROID DIST FOR EACH CLUSTER
        centroid_dist[idx_clust_a] = CalcCentroidDist(clusters[idx_clust_a], *centroids[idx_clust_a]);
        centroid_dist[idx_clust_b] = CalcCentroidDist(clusters[idx_clust_b], *centroids[idx_clust_b]);
    }

    if (clusterPrint) {
        for (int c = 0; c < numClusters; ++c) {                             // PRINT NEW CENTROID COORDS
            printf("\nCentroid: %d\t(%.1f, %.1f)", c, centroids[c]->x, centroids[c]->y);
        }
    }
    if (clusterPrint) cout << "\n-------- ^^kMEANS^^  --------\n";
    vector<Cluster*> clusterss;// = vector<Cluster*>(numClusters);
    for (auto& cluster : clusters) {
        clusterss.push_back(new Cluster(cluster));     // initialise clusterss with clusters (vector of ReefPt pointers
	}
    return clusterss;
}

//ClusterSoln clusterAndCentroid(Problem problem) {
//    //if (!init_solution) {
//    ClusterSoln init_solution(  kMeansConstrained(
//            problem.getReefPointers(), problem.getnumClusters(), 1000, false));
//   // }   // only happens at first initialisation
//   // else { 
//   //     for (auto& cluster : init_solution.getClusters()) { 
//   //         for (auto& reef : cluster->getReefs()) {    // reef is a ReefPt pointer
//			//	init_solution.setClusters(reef->getID(), cluster->getID());
//			//}
//   //         //clusteredPoints.push_back(cluster->getReefs());     //getReefs() returns a vector of Reef pointers
//   //     } 
//   // }       // else: use existing clusters (don't redo kMeans)
//    return init_solution;
//}