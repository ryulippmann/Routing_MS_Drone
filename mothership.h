#pragma once
#include <iostream>
#include <vector>
#include "TSPheuristics_annotated.h"

vector<vector<double>> calc_centMatrix(const vector<ClusterSoln*>& clusters, const Pt depot) {
    vector<vector<double>> centroidMatrix;
    //centroids.insert(centroids.begin(), ClusterPoint(depot.first, depot.second));
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
    return centroidMatrix;
}

vector<ClusterSoln*> clusterCentroidNearestNeighbour(Problem& inst, vector<ClusterSoln*> clusters,//const ClusterSoln& clustSoln,
    /*const Problem inst, */
    //const vector<Cluster*> clusters, const vector<vector<double>>& centroidMatrix,
    bool csvPrint = false, bool printStops = true) {
    vector<ClusterSoln*> nearestCentroids(clusters.size(), nullptr);// Initialize the result vector
    vector<int> visited(clusters.size(), 0);
    int u = -1;                                                  // initialise current index
                    //u[0,3]
    //visited[u] = 1;                                             // mark current index as visited
    const vector<vector<double>> centroidMatrix = calc_centMatrix(clusters, inst.ms.depot);
    for (int c = 0; c < clusters.size(); c++) {                 // for stops within vehicle capacity
        const auto& neighbours = centroidMatrix[u+1];             // for u vector in dMatrix
        double min = DBL_MAX;                                   // initialise min dist as MAX
        int v = -1;                                             // initialise index as -1 = invalid entry

        for (int w = 0; w < clusters.size(); w++) { // for each neighbour of u: w[0,2]
            if (neighbours[w+1] < min && visited[w] == 0) {       // if closer unvisited reef:
                v = w;                                          // update index
                min = neighbours[v+1];                            // update min
            }//if
        }//for
        //init_solution.mothership.route_dist += min;           // add dist travelled to route dist
        u = v;                                                  // Update u=v as new index of closest pt
        visited[u] = 1;                                         // Mark u=v(new) as visited.
        nearestCentroids[c] = clusters[v];                    // Add closest reef to solution
    }//for(v.capacity)
    return nearestCentroids;
}//nearestNeighbour

void greedyMSCluster(Problem& inst, MSSoln& msSoln,
    //const ClusterSoln& clustSoln, //const vector<Cluster*>& clusters,
    bool csv_print = false, bool print = true) {
    if (print) cout << "\n---- GREEDY M/S CLUSTERS ----\n";
    const vector<ClusterSoln*> clusters = msSoln.clusters;
    int n = clusters.size();
    double gd_2opt_dists;
    const vector<vector<double>> centroidMatrix = calc_centMatrix(clusters, inst.ms.depot);

    double route_dist = centroidMatrix[0][clusters[0]->ID + 1];
    for (int i = 1; i < centroidMatrix.size() - 1; i++) {
        route_dist += centroidMatrix[clusters[i - 1]->ID + 1][clusters[i]->ID + 1];
    }//for(i=from_pts)
    route_dist += centroidMatrix[clusters.back()->ID + 1][0];

    vector<int> i_tour(1, 0);                         // ai_tour = city_index for each tour -> nearest neighbour
    for (int i = 0; i < clusters.size(); i++) {
        i_tour.push_back(clusters[i]->ID + 1);
    }//for(i=from_pts)
    i_tour.push_back(0);                                                            // add depot as last stop

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
        //if (print) {
        //    cout << "\n\t\t";
        //    for (const auto& stop : init_solution.mothership.launch_stops) { cout << "\t" << stop; }
        //    cout << "\n\n";
        //}
        vector <ClusterSoln*> temp_clust(clusters.size(), nullptr);
        for (int i = 0; i < clusters.size(); i++) {
		    temp_clust[i] = clusters[gd_out.second[i+1]-1];
	    }

        msSoln.clusters = temp_clust;                        // UPDATE CLUSTER ORDER
    } else if (print) { printf("\tNO IMPROVEMENT\n"); }//else

    //if (print) {
    //    printMSroute(init_solution.mothership, centroids);
    //    cout << "------------- ^^Gd M/S^^ --------------\n";
    //}
    return;     					// updated clustSoln.clusters
}

// warning: this needs to be set before msSoln.getDist() is called
vector<vector<double>> setLaunchPts(Problem& inst, MSSoln& msSoln,
    bool csv_print = false, bool print = true) {
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
        printf("\t%d\t( %.2f, %.2f)\n", inst.ms.depot.ID, inst.ms.depot.x, inst.ms.depot.y);
		for (const auto& stop : msSoln.launchPts) { 
            printf("\t%d\t( %.2f, %.2f)\n", stop->ID, stop->x, stop->y);
        } printf("\n");
        for (int i = 0; i < dMatrix_launchpt.size(); i++) {
            for (int j = 0; j < dMatrix_launchpt[i].size(); j++) {
                printf("\t%.2f", dMatrix_launchpt[i][j]);
            }
            printf("\n");
        }
        cout << string(30, '-') << "\n";
        double total_dist = 0;
        printf("\t%.2f +", dMatrix_launchpt.back()[0]);
        total_dist += dMatrix_launchpt.back()[0];
        for (int i = 0; i < dMatrix_launchpt.size()-1; i++) {
			printf("\t%.2f +", dMatrix_launchpt[i][i+1]);
            total_dist += dMatrix_launchpt[i][i+1];
		}
        printf("\n = %.2f", total_dist);
	}    
	return dMatrix_launchpt;
}

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