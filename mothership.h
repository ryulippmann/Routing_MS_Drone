#pragma once
#include "TSPheuristics_annotated.h"

/// <summary>
/// returns dMatrix between centroids including depot of order c+1
/// </summary>
/// <param name="clusters"></param>
/// <param name="test"></param>
/// <returns></returns>
vector<vector<double>> calc_centMatrix(const Pt& depot, const vector<ClusterSoln*>& clusters, int min_idx, bool test = false) {
    vector<vector<double>> centroidMatrix;
    if (test) {
        vector<ClusterSoln*> clusters_ordered(clusters.size(), nullptr);
        vector<int> route(1,0);

        for (int i = 0; i < clusters.size(); i++) {		// for each cluster
            int clust_ID = clusters[i]->ID - min_idx;
            route.push_back(clust_ID+1);
            clusters_ordered[clust_ID] = clusters[i];
        }
        vector<double> depot_row;
        depot_row.push_back(0);
        for (int i = 0; i < clusters.size(); i++) {		// for each cluster
            depot_row.push_back(calculatePtDistance(depot, clusters_ordered[i]->getCentroid()));
        }
        centroidMatrix.push_back(depot_row);
        for (int i = 0; i < clusters.size(); i++) {		// for each cluster
            vector<double> row(clusters.size() + 1);
            row[0] = depot_row[i + 1];
            for (int j = 0; j < clusters.size(); j++) {	// for each cluster
                row[j + 1] = calculatePtDistance(clusters_ordered[i]->getCentroid(), clusters_ordered[j]->getCentroid());
            }
            centroidMatrix.push_back(row);
        }

    }
    else {
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
    }
    return centroidMatrix;
}

// Function to find index of ClusterSoln* in vector by ClusterSoln->ID
int findClusterByID(int targetID, const vector<ClusterSoln*>& myList) {
    for (int index = 0; index < myList.size(); index++) {
        if (myList[index]->ID == targetID) { return index; }
    }
    throw invalid_argument("ClusterSoln ID not found");
    return -1;  // Return a special value (e.g., -1) to indicate that the ID was not found
}

//void setLaunchPts(MSSoln& msSoln, bool print = false) {
//    const Pt depot = msSoln.ms.depot;
//    if (print) printf("\n---- SET LAUNCH POINTS ----\n\tID\t(  x  ,  y  )\n");
//    if (msSoln.launchPts[0] == nullptr) { // create new objects
//        vector<Pt*> launchPts;
//        launchPts.push_back(new Pt(
//            0.5 * (depot.x + msSoln.clusters[0]->getCentroid().x),
//            0.5 * (depot.y + msSoln.clusters[0]->getCentroid().y))); // depot to first centroid
//        for (int c = 0; c < msSoln.clusters.size() - 1; c++) {
//            launchPts.push_back(new Pt(
//                0.5 * (msSoln.clusters[c]->getCentroid().x + msSoln.clusters[c + 1]->getCentroid().x),
//                0.5 * (msSoln.clusters[c]->getCentroid().y + msSoln.clusters[c + 1]->getCentroid().y))); // centroid to centroid
//        }
//        launchPts.push_back(new Pt(
//            0.5 * (depot.x + msSoln.clusters.back()->getCentroid().x),
//            0.5 * (depot.y + msSoln.clusters.back()->getCentroid().y))); // last centroid to depot
//        msSoln.launchPts = launchPts;
//    }
//    else { // update existing objects
//        msSoln.launchPts[0]->x = 0.5 * (depot.x + msSoln.clusters[0]->getCentroid().x);
//        msSoln.launchPts[0]->y = 0.5 * (depot.y + msSoln.clusters[0]->getCentroid().y); // depot to first centroid
//        for (int c = 0; c < msSoln.clusters.size() - 1; c++) { // sum adjacent clusters x,y's to calc launchpts 
//            msSoln.launchPts[c + 1]->x = 0.5 * (msSoln.clusters[c]->getCentroid().x + msSoln.clusters[c + 1]->getCentroid().x);
//            msSoln.launchPts[c + 1]->y = 0.5 * (msSoln.clusters[c]->getCentroid().y + msSoln.clusters[c + 1]->getCentroid().y);
//        }
//        msSoln.launchPts.back()->x = 0.5 * (depot.x + msSoln.clusters.back()->getCentroid().x);
//        msSoln.launchPts.back()->y = 0.5 * (depot.y + msSoln.clusters.back()->getCentroid().y); // last centroid to depot
//        if (print) {
//            printf("\tID\t(  x  ,  y  )\n");
//            cout << string(30, '-') << "\n";
//            printf("\t%d\t( %2.2f, %2.2f)\n", depot.ID, depot.x, depot.y);
//            for (const auto& stop : msSoln.launchPts) {
//                printf("\t%d\t( %.2f, %.2f)\n", stop->ID, stop->x, stop->y);
//            } printf("\n");
//            cout << string(30, '-') << "\n";
//            double total_dist = calculatePtDistance(depot, msSoln.launchPts[0]);
//            printf("\t%.2f ", total_dist);
//            for (int i = 0; i < msSoln.launchPts.size() - 1; i++) {
//                double leg_dist = calculatePtDistance(msSoln.launchPts[i], msSoln.launchPts[i + 1]);
//                printf("+\t%.2f ", leg_dist);
//                total_dist += leg_dist;
//            }
//            printf("+\t%.2f ", calculatePtDistance(depot, msSoln.launchPts.back()));
//            total_dist += calculatePtDistance(depot, msSoln.launchPts.back());
//            printf("\n\t\t= %.2f", total_dist);
//        }
//    }
//}
void setLaunchPts(MSSoln& msSoln, const pair<double, double>& weights) {
    const Pt depot = msSoln.ms.depot;
    pair<double, double> centroid_weighting = make_pair(1, 2);  // return trip: double drone weight
    // centroid_weighting HARD-CODED = (1,2)

    vector<Pt> centroids;
    for (const auto& clust : msSoln.clusters) {
        centroids.push_back(clust->getCentroid());
    }

    double w_ms = weights.first * centroid_weighting.first;
    double w_d = weights.second * centroid_weighting.second;
    normaliseWeights(w_ms, w_d);
    // this formulation takes into account the following destination, and the 'gravity' towards that point

    if (msSoln.launchPts[0] == nullptr) {
        vector<Pt*> launchPts;
        launchPts.push_back(new Pt(
            w_ms * depot.x + w_d * centroids[0].x,
            w_ms * depot.y + w_d * centroids[0].y)); // depot to first centroid
        for (int c = 0; c < centroids.size() - 1; c++) {
            launchPts.push_back(new Pt(
                w_ms * launchPts[c]->x + w_d * (centroids[c + 1].x),
                w_ms * launchPts[c]->y + w_d * (centroids[c + 1].y))); // centroid to centroid
        }
        launchPts.push_back(new Pt(
            w_ms * launchPts.back()->x + w_d * depot.x,
            w_ms * launchPts.back()->y + w_d * depot.y)); // last centroid to depot
        msSoln.launchPts = launchPts;
    }
	else {
        msSoln.launchPts[0]->x = w_ms * depot.x + w_d * centroids[0].x;
        msSoln.launchPts[0]->y = w_ms * depot.y + w_d * centroids[0].y; // depot to first centroid   
        for (int c = 0; c < msSoln.clusters.size() - 1; c++) {
            msSoln.launchPts[c + 1]->x = w_ms * msSoln.launchPts[c]->x + w_d * (centroids[c + 1].x);
            msSoln.launchPts[c + 1]->y = w_ms * msSoln.launchPts[c]->y + w_d * (centroids[c + 1].y); // centroid to centroid
        }
        msSoln.launchPts.back()->x = w_ms * msSoln.launchPts[msSoln.launchPts.size() - 2]->x + w_d * depot.x;
        msSoln.launchPts.back()->y = w_ms * msSoln.launchPts[msSoln.launchPts.size() - 2]->y + w_d * depot.y; // last centroid to depot
    }
    return;
}
void alt_setLaunchPts(MSSoln& msSoln, const pair<double, double>& weights) {
    const Pt depot = msSoln.ms.depot;
    pair<double, double> centroid_weighting = make_pair(1, 2);  // return trip: double drone weight
    // centroid_weighting HARD-CODED = (1,2)

    vector<Pt> centroids;
    for (const auto& clust : msSoln.clusters) {
        centroids.push_back(clust->getCentroid());
    }

    double w_ms = weights.first * centroid_weighting.first;
    double w_d = weights.second * centroid_weighting.second;
    normaliseWeights(w_ms, w_d);
    // this formulation takes into account the following destination, and the 'gravity' towards that point

    if (msSoln.launchPts[0] == nullptr) {
        vector<Pt*> launchPts;
        launchPts.push_back(new Pt(
            w_ms * depot.x + w_d * centroids[0].x,
            w_ms * depot.y + w_d * centroids[0].y)); // depot to first centroid
        for (int c = 0; c < centroids.size() - 2; c++) {
            launchPts.push_back(new Pt(
                w_ms * launchPts[c]->x + w_d * (centroids[c].x + centroids[c + 1].x + centroids[c + 2].x) / 3,
                w_ms * launchPts[c]->y + w_d * (centroids[c].y + centroids[c + 1].y + centroids[c + 2].y) / 3)); // centroid to centroid
        }
        launchPts.push_back(new Pt(
            w_ms * launchPts.back()->x + w_d * (centroids[centroids.size() - 2].x + 2 * centroids.back().x) / 3,
            w_ms * launchPts.back()->y + w_d * (centroids[centroids.size() - 2].y + 2 * centroids.back().y) / 3)); // last centroid to depot
        launchPts.push_back(new Pt(
            w_ms * launchPts.back()->x + w_d * depot.x,
            w_ms * launchPts.back()->y + w_d * depot.y)); // last centroid to depot
        msSoln.launchPts = launchPts;
    }
    else {
        msSoln.launchPts[0]->x = w_ms * depot.x + w_d * centroids[0].x;
        msSoln.launchPts[0]->y = w_ms * depot.y + w_d * centroids[0].y; // depot to first centroid   
        for (int c = 0; c < msSoln.clusters.size() - 2; c++) {
            msSoln.launchPts[c + 1]->x = w_ms * msSoln.launchPts[c]->x + w_d * (centroids[c].x + centroids[c + 1].x + centroids[c + 2].x) / 3;
            msSoln.launchPts[c + 1]->y = w_ms * msSoln.launchPts[c]->y + w_d * (centroids[c].y + centroids[c + 1].y + centroids[c + 2].y) / 3; // centroid to centroid
        }
        msSoln.launchPts[msSoln.launchPts.size() - 2]->x = w_ms * msSoln.launchPts[msSoln.launchPts.size() - 3]->x + w_d * (centroids[centroids.size() - 2].x + 2 * centroids.back().x) / 3;
        msSoln.launchPts[msSoln.launchPts.size() - 2]->y = w_ms * msSoln.launchPts[msSoln.launchPts.size() - 3]->y + w_d * (centroids[centroids.size() - 2].y + 2 * centroids.back().y) / 3; // last centroid to depot
        
        msSoln.launchPts.back()->x = w_ms * msSoln.launchPts.back()->x + w_d * depot.x;
        msSoln.launchPts.back()->y = w_ms * msSoln.launchPts.back()->y + w_d * depot.y; // last centroid to depot
    }
    return;
}

int findMinIdx(const vector<ClusterSoln*>& clusters) {    //find min ID of all clusters
    int min_ID = INT_MAX;
    for (const auto& clust : clusters) { min_ID = min(min_ID, clust->ID); }
    return min_ID;
}

double clusterCentroidNearestNeighbour(MSSoln& msSoln, const pair<double, double>& weights) {
    const vector<ClusterSoln*> clusters = msSoln.clusters;
    vector<ClusterSoln*> nearestCentroids(clusters.size(), nullptr);// Initialize the result vector
    vector<bool> visited(clusters.size(), 0);
    int u = 0;                                                  // initialise current index
    const vector<vector<double>> centroidMatrix = calc_centMatrix(msSoln.ms.depot, clusters, findMinIdx(clusters));
    if (print_detail) { printf("\n");
		for (int i = 0; i < centroidMatrix.size(); i++) {
			for (int j = 0; j < centroidMatrix[i].size(); j++) { printf("\t%.2f", centroidMatrix[i][j]); }
			printf("\n");
		} printf("\n");
	}
    for (int c = 0; c < clusters.size(); c++) {             // for stops within vehicle capacity
        const auto& neighbours = centroidMatrix[u];         // for u vector in dMatrix
        double min = DBL_MAX;                               // initialise min dist as MAX
        int v = -1;                                         // initialise index as -1 = invalid entry
        for (int w = 1; w < clusters.size()+1; w++) {       // for each neighbour of u: w[0,2]
            if (neighbours[w] < min && visited[w-1] == 0) { // if closer unvisited reef:
                v = w;                                      // update index
                min = neighbours[v];                        // update min
            }//if
        }//for
        u = v;                                              // Update new index of closest pt
        visited[u-1] = 1;                                   // Mark u=v(new) as visited.
        nearestCentroids[c] = clusters[v-1];                  // Add closest reef to solution
        if (print_detail) printf("\t%d\t%.2f\n", v, min);
    }//for(cluster)
    msSoln.clusters = nearestCentroids;				// Update clustSoln.clusters    
    setLaunchPts(msSoln, weights);                           // Update msSoln.launchPts
    double msDist = msSoln.getDist();               // Calc msSoln.dist from depot to launchPts!
    if (print_detail) printf("  =?=\t%.2f\n", msDist);
    return msDist;
}//nearestNeighbour

double greedyMSCluster(MSSoln& msSoln, const pair<double, double>& weights) {
    if (print_detail) printf("\n---- GREEDY M/S CLUSTERS ----\n");
    printf("%.2f\n", msSoln.getDist());
    const vector<ClusterSoln*> clusters = msSoln.clusters;
    int n = clusters.size();
    double gd_2opt_dists;

    int min_clust_idx = findMinIdx(clusters);
    const vector<vector<double>> centroidMatrix = calc_centMatrix(msSoln.ms.depot, clusters, min_clust_idx, true);
    
    // print centroidMatrix
    if (print_detail) {
        printf("\n");
		for (int i = 0; i < centroidMatrix.size(); i++) {
			for (int j = 0; j < centroidMatrix[i].size(); j++) {
				printf("\t%.2f", centroidMatrix[i][j]);
			}
			printf("\n");
		}
	}

    vector<int> i_tour (1, 0);                       // ai_tour = city_index for each tour -> nearest neighbour
    for (int i = 0; i < clusters.size(); i++) {
        i_tour.push_back(clusters[i]->ID + 1 - min_clust_idx);
    }//for(i=from_pts)

    double route_dist = 0;//centroidMatrix[0][clusters[0]->ID + 1];
    if (print_detail) printf("\n");//\t % .2f", route_dist);
    for (int i = 0; i < centroidMatrix.size() - 1; i++) {
        double ddd = centroidMatrix[i_tour[i]][i_tour[i + 1]];
        if (print_detail) printf("\t%.2f", ddd);
        route_dist += ddd;
    }//for(i=from_pts)
    route_dist += centroidMatrix[i_tour.back()][0];
    if (print_detail) printf("\t%.2f", centroidMatrix[i_tour.back()][0]);
    if (print_detail) printf("\n\t%.2f\t == total return dist:depot-centroids\n", route_dist);

    pair<double, vector<int>> gd_out = gd_local_2opt_search(i_tour.size(), centroidMatrix, i_tour, false);      // args = (int ai_n, vector<vector<double>> &ad_dist, vector<int> &ai_tour, bool ab_full_nbrhd)
    gd_2opt_dists = gd_out.first;
    if (print_detail) { printf("Vehicle\t\tInitial\t\tGreedy 2-Opt\n\t\t%7.3f  \t%7.3f\t", route_dist, gd_2opt_dists); }
    
    double improvement = route_dist - gd_2opt_dists;
    if (improvement > 0.0001 * route_dist) {                //if greedy solution is better than current
        if (print_detail) printf("\t%.2f%%\tIMPROVEMENT\t", improvement * 100 / route_dist);
        route_dist = gd_2opt_dists;
        
        // Find the index of 0 in proposed_tour
        vector<int> p_tour = gd_out.second;
        auto it = find(p_tour.begin(), p_tour.end(), 0);
        int idx = it - p_tour.begin();

        // Create the oriented vector
        vector<int> oriented_vector;
        oriented_vector.reserve(p_tour.size());
        for (size_t i = idx; i < p_tour.size(); ++i) { oriented_vector.push_back(p_tour[i]); }
        for (size_t i = 0; i < idx; ++i) { oriented_vector.push_back(p_tour[i]); }

        if (print_detail) {
            printf("\n\t\t");
            for (const auto& stop : oriented_vector) { printf("\t%d", stop); }
            printf("\n\n");
        }

        vector <ClusterSoln*> temp_clust;
        ////find min ID of all clusters
        //int min_ID = INT_MAX;
        //for (const auto& clust : clusters) { min_ID = min(min_ID, clust->ID); }
        for (int i = 1; i < clusters.size() + 1; i++) {
            temp_clust.push_back(clusters[findClusterByID(oriented_vector[i] + min_clust_idx - 1, clusters)]);
	    }
        msSoln.clusters = temp_clust;                        // UPDATE CLUSTER ORDER
    } else if (print_detail) { printf("\tNO IMPROVEMENT\n"); }//else

    setLaunchPts(msSoln, weights);
    double msDist = msSoln.getDist();
    printf("\n%.2f\n", msDist);
    return msDist;
}

///////////////////////////////////////////////////////////////////
/// <summary>
/// return vector of msSolns with dists: initial, NN, and greedy 2-opt
/// </summary>
/// <param name="clusters"></param>
/// <param name="msSoln"></param>
/// <param name="csv_print"></param>
/// <returns></returns>
vector<pair<double, MSSoln>> initMsSoln(const vector<ClusterSoln*>& clusters, MSSoln& msSoln, pair<double, double> weights) {
	vector<pair<double, MSSoln>> msSolns;
    double msDist=DBL_MAX;                      // No launchPts initialised yet
    msSolns.push_back(make_pair(msDist, msSoln));

    msDist = clusterCentroidNearestNeighbour(msSoln, weights);		// clusters ordered by NN
    msSolns.push_back(make_pair(msDist, msSoln));
    msDist = greedyMSCluster(msSoln, weights);						// Improve using Gd 2-Opt: update clustSoln.clustOrder
    msSolns.push_back(make_pair(msDist, msSoln));
    if (print_general) {
        printf("\tMS LaunchPts\n");
        for (const auto& launchPt : msSolns.back().second.launchPts) {
            printf("\t%d\t(%.2f, %.2f)\n", launchPt->ID, launchPt->x, launchPt->y);
        }
        printf("\tDepot:\t(%.2f, %.2f)\tTOTAL MS DIST: %.2f\t-->  WEIGHTED == %.2f\n", msSoln.ms.depot.x, msSoln.ms.depot.y, msSolns.back().first, weights.first*msSolns.back().first);
    }
	return msSolns;
}

///////////////////////////////////////////////////////////////////
