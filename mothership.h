#pragma once
#include "TSPheuristics_annotated.h"

// returns dMatrix between centroids including depot of order c+1
vector<vector<double>> calc_centMatrix(const vector<ClusterSoln*>& clusters, bool test = false) {
    vector<vector<double>> centroidMatrix;
    if (test) {
        vector<ClusterSoln*> clusters_ordered(clusters.size(), nullptr);
        vector<int> route(1,0);
        for (int i = 0; i < clusters.size(); i++) {		// for each cluster
            int clust_ID = clusters[i]->ID;
            route.push_back(clust_ID+1);
            clusters_ordered[clust_ID] = clusters[i];
        }
        vector<double> depot_row;
        depot_row.push_back(0);
        for (int i = 0; i < clusters.size(); i++) {		// for each cluster
            depot_row.push_back(calculatePtDistance(inst.ms.depot, clusters_ordered[i]->getCentroid()));
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
            depot_row.push_back(calculatePtDistance(inst.ms.depot, clusters[i]->getCentroid()));
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
//		depot_row.push_back(calculatePtDistance(inst.ms.depot, *launchPts[i]));
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

// warning: this needs to be set before msSoln.getDist() is called
// returns dMatrix betweem launchPts including depot
// c clusters -> c+1 launchPts including depot
vector<vector<double>> setLaunchPts(MSSoln& msSoln, bool print = false) {
    const vector<ClusterSoln*> clusters = msSoln.clusters;
    if (print) printf("\n---- SET LAUNCH POINTS ----\n");
    vector<Pt*> launchPts;
    //launchPts.push_back(new Pt(inst.ms.depot));       // add depot as first launch point
    launchPts.push_back(new Pt(
        (inst.ms.depot.x + clusters[0]->getCentroid().x)/2,
        (inst.ms.depot.y + clusters[0]->getCentroid().y)/2));
	for (int c = 0; c < clusters.size()-1; c++) {
        launchPts.push_back(new Pt(        // sum adjacent clusters x,y's to calc launchpts and 
                (clusters[c]->getCentroid().x + clusters[c+1]->getCentroid().x) / 2,
                (clusters[c]->getCentroid().y + clusters[c+1]->getCentroid().y) / 2));
	}
    launchPts.push_back(new Pt(
        (inst.ms.depot.x + clusters.back()->getCentroid().x) / 2,
        (inst.ms.depot.y + clusters.back()->getCentroid().y) / 2));
    //launchPts.push_back(new Pt(inst.ms.depot));       // add depot as first launch point
	msSoln.launchPts = launchPts;
    vector<vector<double>> dMatrix_launchpt = msSoln.launchPt_dMatrix();
	if (print) {
        printf("\tID\t(  x  ,  y  )\n");
        cout << string(30, '-') << "\n";
        printf("\t%d\t( %2.2f, %2.2f)\n", inst.ms.depot.ID, inst.ms.depot.x, inst.ms.depot.y);
		for (const auto& stop : msSoln.launchPts) { 
            printf("\t%d\t( %.2f, %.2f)\n", stop->ID, stop->x, stop->y);
        } printf("\n");
        for (int i = 0; i < dMatrix_launchpt.size(); i++) {
            for (int j = 0; j < dMatrix_launchpt[i].size(); j++) {
                printf("\t%.2f", dMatrix_launchpt[i][j]);
            }
            printf("\n");
        } printf("\n");
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

double clusterCentroidNearestNeighbour(MSSoln& msSoln, bool print = true) {
    const vector<ClusterSoln*> clusters = msSoln.clusters;
    vector<ClusterSoln*> nearestCentroids(clusters.size(), nullptr);// Initialize the result vector
    vector<bool> visited(clusters.size(), 0);
    int u = -1;                                                  // initialise current index
    const vector<vector<double>> centroidMatrix = calc_centMatrix(clusters);
    if (print) { printf("\n");
		for (int i = 0; i < centroidMatrix.size(); i++) {
			for (int j = 0; j < centroidMatrix[i].size(); j++) { printf("\t%.2f", centroidMatrix[i][j]); }
			printf("\n");
		} printf("\n");
	}
    for (int c = 0; c < clusters.size(); c++) {             // for stops within vehicle capacity
        const auto& neighbours = centroidMatrix[u+1];       // for u vector in dMatrix
        double min = DBL_MAX;                               // initialise min dist as MAX
        int v = -1;                                         // initialise index as -1 = invalid entry
        for (int w = 0; w < clusters.size(); w++) {         // for each neighbour of u: w[0,2]
            if (neighbours[w+1] < min && visited[w] == 0) { // if closer unvisited reef:
                v = w;                                      // update index
                min = neighbours[v+1];                      // update min
            }//if
        }//for
        u = v;                                              // Update new index of closest pt
        visited[u] = 1;                                     // Mark u=v(new) as visited.
        nearestCentroids[c] = clusters[v];                  // Add closest reef to solution
        if (print) printf("\t%d\t%.2f\n", v, min);
    }//for(cluster)
    msSoln.clusters = nearestCentroids;				// Update clustSoln.clusters    
    setLaunchPts(msSoln, true);                           // Update msSoln.launchPts
    double msDist = msSoln.getDist();               // Calc msSoln.dist from depot to launchPts!
    printf("  =?=\t%.2f\n", msDist);
    return msDist;
}//nearestNeighbour

double greedyMSCluster(MSSoln& msSoln, bool print = true) {
    if (print) cout << "\n---- GREEDY M/S CLUSTERS ----\n";
    printf("%.2f\n", msSoln.getDist());
    const vector<ClusterSoln*> clusters = msSoln.clusters;
    int n = clusters.size();
    double gd_2opt_dists;
    const vector<vector<double>> centroidMatrix = calc_centMatrix(clusters, true);
    const vector<vector<double>> launchPtMatrix = setLaunchPts(msSoln, print);
    // print centroidMatrix
    if (print) {
		cout << "\n";
		for (int i = 0; i < centroidMatrix.size(); i++) {
			for (int j = 0; j < centroidMatrix[i].size(); j++) {
				printf("\t%.2f", centroidMatrix[i][j]);
			}
			printf("\n");
		}
	}

    //double total_dist = 0;
    //printf("\t%.2f ", dMatrix_launchpt.back()[0]);
    //total_dist += dMatrix_launchpt.back()[0];
    //for (int i = 0; i < dMatrix_launchpt.size() - 1; i++) {
    //    printf("+\t%.2f ", dMatrix_launchpt[i][i + 1]);
    //    total_dist += dMatrix_launchpt[i][i + 1];
    //}
    //printf("\n\t\t= %.2f", total_dist);

    vector<int> i_tour (1, 0);                       // ai_tour = city_index for each tour -> nearest neighbour
    for (int i = 0; i < clusters.size(); i++) {
        i_tour.push_back(clusters[i]->ID + 1);
    }//for(i=from_pts)
    //i_tour.push_back(0);                            // add depot as last stop

    double route_dist = 0;//centroidMatrix[0][clusters[0]->ID + 1];
    if (print) printf("\n");//\t % .2f", route_dist);
    for (int i = 0; i < centroidMatrix.size() - 1; i++) {
        double ddd = centroidMatrix[i_tour[i]][i_tour[i + 1]];
            //[i + 1];
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
        /* CLOSE AND ORIEND CLUSTER LOOP!*/
        //vector<int> d = closeandOrientClusterLoop(gd_out, init_solution.mothership, init_solution.clustOrder);
        //init_solution.mothership.launch_stops = d;//UPDATE ROUTE LIST
        //init_solution.clustOrder = make_pair(init_solution.mothership.launch_stops, gd_out.first);
        
        // Find the index of 0 in proposed_tour
        vector<int> p_tour = gd_out.second;
        auto it = find(p_tour.begin(), p_tour.end(), 0);
        int idx = it - p_tour.begin();

        // Create the oriented vector
        vector<int> oriented_vector;
        oriented_vector.reserve(p_tour.size());
        for (size_t i = idx; i < p_tour.size(); ++i) {
            oriented_vector.push_back(p_tour[i]);
        }
        for (size_t i = 0; i < idx; ++i) {
            oriented_vector.push_back(p_tour[i]);
        }

        if (print) {
            cout << "\n\t\t";
            for (const auto& stop : oriented_vector) { cout << "\t" << stop; }
            cout << "\n\n";
        }

        vector <ClusterSoln*> temp_clust;
        for (int i = 1; i < clusters.size() + 1; i++) {
            temp_clust.push_back(clusters[findClusterByID(oriented_vector[i] - 1, clusters)]);
	    }
        msSoln.clusters = temp_clust;                        // UPDATE CLUSTER ORDER
    } else if (print) { printf("\tNO IMPROVEMENT\n"); }//else

    setLaunchPts(msSoln, print);
    double msDist = msSoln.getDist();
    printf("\n%.2f\n", msDist);
    return msDist;
}

///////////////////////////////////////////////////////////////////

vector<pair<double, MSSoln>> initMsSoln(const vector<ClusterSoln*>& clusters, MSSoln& msSoln, bool csv_print=0) {
	vector<pair<double, MSSoln>> msSolns;
    double msDist=DBL_MAX;                      // No launchPts initialised yet
    msSolns.push_back(make_pair(msDist, msSoln));

    msDist = clusterCentroidNearestNeighbour(msSoln);		// clusters ordered by NN
    msSolns.push_back(make_pair(msDist, msSoln));
    if (csv_print) csvPrintMSRoutes(msSoln.launchPts, "ms_launch_route_NN", msDist);
    msDist = greedyMSCluster(msSoln);						// Improve using Gd 2-Opt: update clustSoln.clustOrder
    msSolns.push_back(make_pair(msDist, msSoln));
    if (csv_print) csvPrintMSRoutes(msSoln.launchPts, "ms_launch_route_Gd", msDist);
    //if (csv_print) csvPrintClusters(msSoln.clusters, "clusters_ordered", kMeansIters);		// CSV PRINT clusters //

	//for (const auto& cluster : clusters) {
	//	MSSoln msSoln(cluster);
	//	msSoln.setLaunchPts();
	//	msSolns.push_back(make_pair(msSoln.getDist(), msSoln));
	//}
	return msSolns;
}

///////////////////////////////////////////////////////////////////

//vector<Pt*> setLaunchPts(const vector<Cluster*>& clusters) {
//	vector<Pt*> launchPts;
//	for (const auto& cluster : clusters) {
//		launchPts.push_back(cluster->getCentroid());
//	}
//	return launchPts;
//}

//vector<int> order_clusters(vector<vector<double>> centroidMatrix) {
//	vector<int> order;
//	vector<double> dist;
//	for (int i = 0; i < centroidMatrix.size(); i++) {
//		dist.push_back(0);
//		for (int j = 0; j < centroidMatrix.size(); j++) {
//			dist[i] += centroidMatrix[i][j];
//		}
//	}
//	for (int i = 0; i < centroidMatrix.size(); i++) {
//		int min = 0;
//		for (int j = 0; j < centroidMatrix.size(); j++) {
//			if (dist[j] < dist[min]) {
//				min = j;
//			}
//		}
//		order.push_back(min);
//		dist[min] = 100000000;
//	}
//	return order;
//}