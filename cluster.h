#pragma once
#include <functional>
// all of these funcitons called once only, but clean up main.cpp

void randomSwapClusters(vector<Pt*>& clust_a, vector<Pt*>& clust_b, const pair<double, double> dists, pair<Pt, Pt> centroids, bool swap_print = false) {
    uniform_int_distribution<int> dist(0, min(clust_a.size(), clust_b.size()) - 1);     //!constrict swaps between launch/retrieve nodes[1,9] (for now)
    int index1 = dist(gen);
    int index2 = dist(gen);

    if (swap_print) {
        printf("\nSwap element %d in cluster a",  index1);
        for (auto& node : clust_a) { printf("\n\t%d:\t\t(%.1f, %.1f)", node->ID, node->x, node->y); }
        printf("\nwith element %d in cluster b\t",  index2);
        for (auto& node : clust_b) { printf("\n\t%d:\t\t(%.1f, %.1f)", node->ID, node->x, node->y); }
    }
    swap(clust_a[index1], clust_b[index2]);     // swap nodes between clusters

    vector<double> centroid_dist(2);        // 0 = clust_a, 1 = clust_b
    centroid_dist[0] = CalcCentroidDist(clust_a, centroids.first);
    centroid_dist[1] = CalcCentroidDist(clust_b, centroids.second);

    if (sum(centroid_dist) > dists.first + dists.second) {
        swap(clust_a[index1], clust_b[index2]);     // if condition met, swap BACK!
    } else if (swap_print) {                        // if condition not met, print new clusters
            printf("\nNow element %d in cluster a",  index1);
            for (const auto& node : clust_a) { printf("\n\t%d:\t\t(%.1f, %.1f)", node->ID, node->x, node->y); }
            printf("\nis element %d in cluster b\t",  index2);
            for (const auto& node : clust_b) { printf("\n\t%d:\t\t(%.1f, %.1f)", node->ID, node->x, node->y); }
    }
    return;
}

vector<ClusterSoln*> kMeansConstrained(int maxIterations, vector<Pt*> points, const int numClusters,
    bool clusterPrint = false, int randomSeed = 12345) { //10^6 iter ~ 8-9 sec. 10^7 iter ~ 3 mins
    if (clusterPrint) printf("\n---- kMEANS CONSTRAINED ----\n");
    int numPoints = points.size();

    vector<ClusterSoln*> clusterss;// = vector<Cluster*>(numClusters);
    // Check if desired cluster size is feasible
    if (numPoints % numClusters != 0) { throw invalid_argument("Clusters don't divide evenly..."); return clusterss; }
    int cluster_size = numPoints / numClusters;
    
    vector<vector<Pt*>> clusters(numClusters);
    vector<Pt*> centroids(numClusters);     // initialised empty. Populate with ClusterPoints             //= getClusterCentroids(clusters, true);
    vector<double> centroid_dist(numClusters);

    for (int c = 0; c < numClusters; c++) { // populate clusters with vector by cluster of vector of Reef Pts in cluster
        for (int i = 0; i < cluster_size; i++) {    // i = node id in cluster
            clusters[c].push_back(points[c * cluster_size + i]); // c*cluster_size = first node id in cluster
        }
        if (clusterPrint || maxIterations) {
            centroids[c] = new Pt(calcCentroid(clusters[c]));
            if (clusterPrint) printf("Centroid: %d\t(%.1f, %.1f)\n", c, centroids[c]->x, centroids[c]->y);
            if (maxIterations) centroid_dist[c] = CalcCentroidDist(clusters[c], *centroids[c]);
        }
    }

    uniform_int_distribution<int> dist(0, clusters.size() - 1);     //!constrict swaps between launch/retrieve nodes[1,9] (for now)
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        int idx_clust_a = dist(gen);
        int idx_clust_b = dist(gen);
        while (idx_clust_a == idx_clust_b) { idx_clust_b = dist(gen);/*continue;*/ }
        if (clusterPrint) printf("\nswap clusters %d -> %d", idx_clust_a, idx_clust_b);

        randomSwapClusters(clusters[idx_clust_a], clusters[idx_clust_b], make_pair(centroid_dist[idx_clust_a], centroid_dist[idx_clust_b]), make_pair(*centroids[idx_clust_a], *centroids[idx_clust_b])/*, iteration*/);

        //UPDATE CENTROIDS!!
        centroids[idx_clust_a] = new Pt(calcCentroid(clusters[idx_clust_a]));
        centroids[idx_clust_b] = new Pt(calcCentroid(clusters[idx_clust_b]));
        //UPDATE TOTAL CENTROID DIST FOR EACH CLUSTER
        centroid_dist[idx_clust_a] = CalcCentroidDist(clusters[idx_clust_a], *centroids[idx_clust_a]);
        centroid_dist[idx_clust_b] = CalcCentroidDist(clusters[idx_clust_b], *centroids[idx_clust_b]);
    }

    if (clusterPrint) {                         // PRINT NEW CENTROID COORDS
        for (int c = 0; c < numClusters; ++c) {
            printf("\nCentroid: %d\t(%.1f, %.1f)", c, centroids[c]->x, centroids[c]->y);
        }
        printf("\n-------- ^^kMEANS^^  --------\n");
    }

    for (auto& cluster : clusters) {
        clusterss.push_back(new ClusterSoln(cluster));     // initialise clusterss with clusters (vector of ReefPt pointers
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