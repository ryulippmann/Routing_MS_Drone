#pragma once
#include "TSPheuristics_annotated.h"

/// <summary>
/// returns dMatrix between centroids including depot of order c+1
/// </summary>
/// <param name="clusters"></param>
/// <param name="test"></param>
/// <returns></returns>
vector<vector<double>> calc_centMatrix(const Pt depot, const vector<ClusterSoln*>& clusters, int min_idx, bool test = false) {
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

//// returns dMatrix between launchPts including depot of order 1+(c+1 launchpts) 
//vector<vector<double>> calc_launchPtMatrix(const vector<Pt*>& launchPts) {
//	vector<vector<double>> launchPtMatrix;
//    vector<double> depot_row;
//    depot_row.push_back(0);
//    for (int i = 0; i < launchPts.size(); i++) {		// for each cluster
//		depot_row.push_back(calculatePtDistance(INST.ms.depot, *launchPts[i]));
//	}
//    launchPtMatrix.push_back(depot_row);
//    for (int i = 0; i < launchPts.size(); i++) {		// for each cluster
//        vector<double> row(launchPts.size() + 1);
//        row[0] = depot_row[i + 1];
//        for (int j = 0; j < launchPts.size(); j++) {	// for each cluster
//			row[j + 1] = calculatePtDistance(*launchPts[i], *launchPts[j]);
//		}
//        launchPtMatrix.push_back(row);
//	}
//    return launchPtMatrix;
//}

// Function to find index of ClusterSoln* in vector by ClusterSoln->ID
int findClusterByID(int targetID, const vector<ClusterSoln*>& myList) {
    for (int index = 0; index < myList.size(); index++) {
        if (myList[index]->ID == targetID) { return index; }
    }
    throw invalid_argument("ClusterSoln ID not found");
    return -1;  // Return a special value (e.g., -1) to indicate that the ID was not found
}

vector<Pt*> SetWeightedLaunchPts(const vector<ClusterSoln*>& clusters, Pt depot, pair<double, double> weights) {
    /*ALT CALCS*/
    ////double w_ms_d = INST.weights.first / (INST.weights.first + INST.weights.second);
    ////vector<Pt*> launchPts;
    ////launchPts.push_back(new Pt(
    ////    w_ms_d * (INST.ms.depot.x + clusters[0]->getCentroid().x),
    ////    w_ms_d * (INST.ms.depot.y + clusters[0]->getCentroid().y)));
    ////for (int c = 0; c < clusters.size() - 1; c++) {
    ////    launchPts.push_back(new Pt(        // sum adjacent clusters x,y's to calc launchpts and 
    ////        w_ms_d * (clusters[c]->getCentroid().x + clusters[c + 1]->getCentroid().x),
    ////        w_ms_d * (clusters[c]->getCentroid().y + clusters[c + 1]->getCentroid().y)));
    ////}
    ////launchPts.push_back(new Pt(
    ////    w_ms_d * (INST.ms.depot.x + clusters.back()->getCentroid().x),
    ////    w_ms_d * (INST.ms.depot.y + clusters.back()->getCentroid().y)));
    // 
    //double w_ms_d = INST.weights.first / (INST.weights.first + INST.weights.second);
    //vector<Pt*> launchPts;
    //launchPts.push_back(new Pt(
    //    w_ms_d * (INST.ms.depot.x + clusters[0]->getCentroid().x),
    //    w_ms_d * (INST.ms.depot.y + clusters[0]->getCentroid().y)));
    //for (int c = 0; c < clusters.size() - 1; c++) {
    //    launchPts.push_back(new Pt(        // sum adjacent clusters x,y's to calc launchpts and 
    //        w_ms_d * (launchPts[c]->x + clusters[c + 1]->getCentroid().x/*launchPts[c+1]->x*/),
    //        w_ms_d * (launchPts[c]->y + clusters[c + 1]->getCentroid().y/*launchPts[c+1]->y*/)));
    //}
    //launchPts.push_back(new Pt(
    //    w_ms_d * (INST.ms.depot.x + launchPts.back()->x),
    //    w_ms_d * (INST.ms.depot.y + launchPts.back()->y)));
    
    double w_ms = weights.first / (weights.first + 2 * weights.second);
    double w_d = weights.second / (weights.first + 2 * weights.second);
    // this formulation takes into account the following destination, and the 'gravity' towards that point
    vector<Pt*> launchPts;
    launchPts.push_back(new Pt(
        w_ms * depot.x + 2 * w_d * clusters[0]->getCentroid().x,
        w_ms * depot.y + 2 * w_d * clusters[0]->getCentroid().y));
    for (int c = 0; c < clusters.size() - 2; c++) {
        launchPts.push_back(new Pt(
            w_ms * launchPts[c]->x + w_d * (clusters[c + 1]->getCentroid().x + clusters[c]->getCentroid().x)/*launchPts[c+1]->x*/,
            w_ms * launchPts[c]->y + w_d * (clusters[c + 1]->getCentroid().y + clusters[c]->getCentroid().x)/*launchPts[c+1]->y*/));
    }
    launchPts.push_back(new Pt(
        w_ms * launchPts.back()->x + w_d * (clusters[clusters.size() - 2]->getCentroid().x + clusters.back()->getCentroid().x)/*launchPts[c+1]->x*/,
        w_ms * launchPts.back()->y + w_d * (clusters[clusters.size() - 2]->getCentroid().y + clusters.back()->getCentroid().x)/*launchPts[c+1]->y*/));

    launchPts.push_back(new Pt(
        w_ms * depot.x + 2 * w_d * launchPts.back()->x,
        w_ms * depot.y + 2 * w_d * launchPts.back()->y));
    return launchPts;
}

/// <summary>
/// warning: this needs to be set before msSoln.getDist() is called
/// returns dMatrix betweem launchPts including depot
/// c clusters -> c+1 launchPts including depot
/// </summary>
/// <param name="msSoln"></param>
/// <param name="print"></param>
/// <returns></returns>
vector<vector<double>> setLaunchPts(MSSoln& msSoln, pair<double, double> weights, bool print = false) {
    const vector<ClusterSoln*> clusters = msSoln.clusters;
    if (print) printf("\n---- SET LAUNCH POINTS ----\n");

    vector<Pt*> launchPts = SetWeightedLaunchPts(clusters, msSoln.ms.depot, weights);

    msSoln.launchPts = launchPts;
    vector<vector<double>> dMatrix_launchpt = msSoln.launchPt_dMatrix();
	if (print) {
        printf("\tID\t(  x  ,  y  )\n");
        cout << string(30, '-') << "\n";
        printf("\t%d\t( %2.2f, %2.2f)\n", msSoln.ms.depot.ID, msSoln.ms.depot.x, msSoln.ms.depot.y);
		for (const auto& stop : msSoln.launchPts) { 
            printf("\t%d\t( %.2f, %.2f)\n", stop->ID, stop->x, stop->y);
        } 
        printf("\n");
        for (int i = 0; i < dMatrix_launchpt.size(); i++) {
            for (int j = 0; j < dMatrix_launchpt[i].size(); j++) {
                printf("\t%.2f", dMatrix_launchpt[i][j]);
            }
            printf("\n");
        } 
        printf("\n");
        cout << string(30, '-') << "\n";
        
        double total_dist = dMatrix_launchpt.back()[0];
        printf("\t%.2f ", total_dist);
        for (int i = 0; i < dMatrix_launchpt.size()-1; i++) {
			printf("+\t%.2f ", dMatrix_launchpt[i][i+1]);
            total_dist += dMatrix_launchpt[i][i+1];
		}
        printf("\n\t\t= %.2f", total_dist);
	}    
	return dMatrix_launchpt;
}

int findMinIdx(const vector<ClusterSoln*>& clusters) {    //find min ID of all clusters
    int min_ID = INT_MAX;
    for (const auto& clust : clusters) { min_ID = min(min_ID, clust->ID); }
    return min_ID;
}

double clusterCentroidNearestNeighbour(MSSoln& msSoln, pair<double, double> weights, bool print = true) {
    const vector<ClusterSoln*> clusters = msSoln.clusters;
    vector<ClusterSoln*> nearestCentroids(clusters.size(), nullptr);// Initialize the result vector
    vector<bool> visited(clusters.size(), 0);
    int u = 0;                                                  // initialise current index
    const vector<vector<double>> centroidMatrix = calc_centMatrix(msSoln.ms.depot, clusters, findMinIdx(clusters));
    if (print) { printf("\n");
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
        if (print) printf("\t%d\t%.2f\n", v, min);
    }//for(cluster)
    msSoln.clusters = nearestCentroids;				// Update clustSoln.clusters    
    setLaunchPts(msSoln, weights, print);                           // Update msSoln.launchPts
    double msDist = msSoln.getDist();               // Calc msSoln.dist from depot to launchPts!
    printf("  =?=\t%.2f\n", msDist);
    return msDist;
}//nearestNeighbour

double greedyMSCluster(MSSoln& msSoln, pair<double, double> weights, bool print = true) {
    if (print) printf("\n---- GREEDY M/S CLUSTERS ----\n");
    printf("%.2f\n", msSoln.getDist());
    const vector<ClusterSoln*> clusters = msSoln.clusters;
    int n = clusters.size();
    double gd_2opt_dists;

    int min_clust_idx = findMinIdx(clusters);

    const vector<vector<double>> centroidMatrix = calc_centMatrix(msSoln.ms.depot, clusters, min_clust_idx, true);
    //const vector<vector<double>> launchPtMatrix = setLaunchPts(msSoln, print);
    // print centroidMatrix
    if (print) {
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
    //i_tour.push_back(0);                            // add depot as last stop

    double route_dist = 0;//centroidMatrix[0][clusters[0]->ID + 1];
    if (print) printf("\n");//\t % .2f", route_dist);
    for (int i = 0; i < centroidMatrix.size() - 1; i++) {
        double ddd = centroidMatrix[i_tour[i]][i_tour[i + 1]];
        if (print) printf("\t%.2f", ddd);
        route_dist += ddd;
    }//for(i=from_pts)
    route_dist += centroidMatrix[i_tour.back()][0];
    if (print) printf("\t%.2f", centroidMatrix[i_tour.back()][0]);
    if (print) printf("\n\t%.2f\t == total return dist:depot-centroids\n", route_dist);

    pair<double, vector<int>> gd_out = gd_local_2opt_search(i_tour.size(), centroidMatrix, i_tour, false);      // args = (int ai_n, vector<vector<double>> &ad_dist, vector<int> &ai_tour, bool ab_full_nbrhd)
    gd_2opt_dists = gd_out.first;
    if (print) { printf("Vehicle\t\tInitial\t\tGreedy 2-Opt\n\t\t%7.3f  \t%7.3f\t", route_dist, gd_2opt_dists); }
    
    double improvement = route_dist - gd_2opt_dists;
    if (improvement > 0.0001 * route_dist) {                //if greedy solution is better than current
        if (print) printf("\t%.2f%%\tIMPROVEMENT\t", improvement * 100 / route_dist);
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

        if (print) {
            printf("\n\t\t");
            for (const auto& stop : oriented_vector) { printf("\t", stop); }
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
    } else if (print) { printf("\tNO IMPROVEMENT\n"); }//else

    setLaunchPts(msSoln, weights, print);
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
vector<pair<double, MSSoln>> initMsSoln(const vector<ClusterSoln*>& clusters, MSSoln& msSoln, pair<double, double> weights, bool print_detail = 0) {
	vector<pair<double, MSSoln>> msSolns;
    double msDist=DBL_MAX;                      // No launchPts initialised yet
    msSolns.push_back(make_pair(msDist, msSoln));

    msDist = clusterCentroidNearestNeighbour(msSoln, weights, print_detail);		// clusters ordered by NN
    msSolns.push_back(make_pair(msDist, msSoln));
    msDist = greedyMSCluster(msSoln, weights, print_detail);						// Improve using Gd 2-Opt: update clustSoln.clustOrder
    msSolns.push_back(make_pair(msDist, msSoln));
    if (!print_detail) {
        printf("\tMS LaunchPts\n");
        for (const auto& launchPt : msSolns.back().second.launchPts) {
            printf("\t%d\t(%.2f, %.2f)\n", launchPt->ID, launchPt->x, launchPt->y);
        }
        printf("\tDepot:\t(%.2f, %.2f)\tTOTAL MS DIST: %.2f\t-->  WEIGHTED == %.2f\n", msSoln.ms.depot.x, msSoln.ms.depot.y, msSolns.back().first, weights.first*msSolns.back().first);
    }
	return msSolns;
}

///////////////////////////////////////////////////////////////////
