#pragma once
//#include <vector>
//#include <cmath>
//using namespace std;

//// RETURN reefs visited
//int reefsVisited(const vector<Pt*>& reefs) { 
//    int visit_count = 0;
//    vector<bool> visited(reefs.size(), false);
//    for (int i = 0; i < reefs.size(); i++) {
//        if (visited[i] == true) { visit_count++; }
//    }
//    return visit_count;
//}//reefsVisited

//vehicles argument not referenced, so are not updated...
vector<vector<Pt*>> TenderWithinClusterNearestNeighbour(const MSSoln& ms, const int c,
    bool csv_print = false) {
    //const Problem& inst = ms.inst;
    const ClusterSoln* cluster = ms.clusters[c];
    pair<Pt*, Pt*> launchPts = make_pair(ms.launchPts[c], ms.launchPts[c + 1]);
    //int reefs_visit_count = reefsVisited(cluster->reefs);    // int of number of reefs visited
    int u = -1;                                          // initialise current reef index
    vector <double> route_dists;//(vehicles.size(),0.0); // list of route dist for each vehicle
    //vector<vector<int>> route_stops;                     // create routes to output to csv
    bool printStops = false;
    if (printStops) { cout << "\n\nNEAREST NEIGHBOUR\n"; }
    vector<vector<Pt*>> routes(inst.tenders.size(), vector<Pt*>(inst.tenderCap, nullptr)); // create routes to save in TenderSoln.routes
    vector<bool> visited(cluster->reefs.size(), false);
    const vector<vector<double>> dMatrix = cluster->getdMatrix(make_pair(ms.launchPts[c], ms.launchPts[c+1]));//[u];                        // for u vector in dMatrix

    for (int m = 0; m < inst.tenders.size() /*&& reefs_visit_count < cluster->reefs.size() - 2*/; m++) {     //FOR EACH VEHICLE!!
        vector<Pt*> row;                            // create row for routes
        // add drop off pt to row
        double route_dist = 0;
        if (row/*cluster.drones[m].reef_stops*/.size() == 0) {                       // if new vehicle = no reefs assigned to vehicle
            row.push_back(launchPts.first);
            u = 0;
        }//if(v.stops==0)       vv      // 3: Find the shortest edge connecting current vertex u and an unvisited vertex v.
        for (int k = 0; k < inst.tenders[m].cap/* && reefs_visit_count < cluster->reefs.size()*/; k++) {            // for stops within vehicle capacity
            const vector<double>& neighbours = dMatrix[u];//cluster->getdMatrix(m, make_pair(ms->launchPts[m], ms->launchPts[m]));//[u];                        // for u vector in dMatrix
            double min = DBL_MAX;                                       // initialise min dist as MAX
            int v = -1;                                                 // initialise index as -1

            for (int w = 1; w < cluster->reefs.size()+1; w++) {
                if (neighbours[w] < min && visited[w-1] == false /*&& cluster->reefs[w].id != -1 && cluster->reefs[w].id != -2*/) { // if closer unvisited reef:
                    v = w;                                              // update index
                    min = neighbours[v];                                // update min
                }//if
            }//for

            route_dist += min;                              // add dist travelled to route dist
            u = v;                                                      // Update u=v as new index of closest pt
            visited[u-1] = true;                                    // Mark u=v(new) as visited.
            row.push_back(cluster->reefs[u-1]);
        }//for(v.capacity)
        // when one stop left, go to pick up pt
        int v = cluster->reefs.size() + 1;
        route_dist += dMatrix[u][v];
        row.push_back(launchPts.second);

        if (printStops) { cout << "Vehicle: " << m << "\tStops:\t"; // Iterate through vehicle count
        printf("\n\tDist:    \t%f\t", route_dist); //}
        cout << "Return from final node: " << row.back()->ID << "\t(" << row.back()->x << ", " << row.back()->y << ")\tback to node: "; 
        }
        row.push_back(row[0]);                  //append first stop to end of "row"
        if (printStops) { cout << row.back() << "\tdone!\n"; }

        // find dMatrix index of 2nd last and last stops, add distance to route_dist
        // this dist should be ZERO because it's from drop to pick nodes...
        route_dist += dMatrix.back()[0];

        routes[m] = row;
        route_dists.push_back(route_dist);                  // append distance of vehicle route
        if (printStops) { cout << "Closed loop dist:\t" << route_dist << "\n\n"; }
    }//for(vehicles)

    if (printStops) { cout << "\nTotal vehicle route dist: " << sum(route_dists) << "\n"; }
    return routes;
}//nearestNeighbour

vector<TenderSoln> initTenderSoln(vector<ClusterSoln*> clusters, MSSoln msSoln) {
    vector<TenderSoln> tenderSolns;
    cout << string(30, '-') << "\n";
    for (int c = 0; c < clusters.size(); c++) {		// for each cluster
        pair<Pt*, Pt*> launchPts = make_pair(msSoln.launchPts[c], msSoln.launchPts[c + 1]);		// launchPts for cluster

        Pt centroid = msSoln.clusters[c]->getCentroid();										// centroid for cluster	
        printf("\nCluster %d\tcentroid: (%.2f, %.2f)\n", msSoln.clusters[c]->ID, centroid.x, centroid.y);

        // print distance matrix
        vector<vector<double>> clusterMatrix = msSoln.clusters[c]->getdMatrix(launchPts);	// distance matrix for cluster
        for (int i = 0; i < clusterMatrix.size(); i++) { for (int j = 0; j < clusterMatrix[i].size(); j++) { printf("%.2f\t", clusterMatrix[i][j]); } printf("\n"); } printf("\n");

        //\\//\\//\\//\\// Initialise TenderSoln  //\\//\\//\\//\\//
        // Tendersoln Nearest Neighbour
        //vector<vector<Pt*>> tenderRoutes = TenderWithinClusterNearestNeighbour(msSoln, c);
        TenderSoln tenderSoln(*msSoln.clusters[c],
            TenderWithinClusterNearestNeighbour(msSoln, c)/*tenderRoutes*/,
            launchPts);
        //\\//\\//\\//\\// TenderSoln Initialised //\\//\\//\\//\\//

        //// Tendersoln Greedy 2-Opt update
        //tenderSoln.routes = greedyTenderCluster(tenderSoln, clusterMatrix);

        // FIXED - deep copy operator: printf("\nERROR HERE - main L97 - ADDING tenderSoln to tenderSolns...\nIs this line necessary/doing anything?\n");
        tenderSolns.emplace_back(tenderSoln);
        // print routes
        printf("CLUSTER %d:\tID:%d\n", c, tenderSoln.cluster.ID);
        for (int i = 0; i < tenderSoln.routes.size(); i++) {
            printf("\t\tRoute %d:\n", i);
            for (int j = 0; j < tenderSoln.routes[i].size(); j++) {
                printf("\t\t\t%d\t(%.2f, %.2f)\n", tenderSoln.routes[i][j]->ID, tenderSoln.routes[i][j]->x, tenderSoln.routes[i][j]->y);
            }// for each node in route
        }// for each route
        cout << string(30, '-') << "\n";
    }// for each cluster
    return tenderSolns;
}


vector<vector<Pt*>> greedyTenderCluster(const TenderSoln& clustTendersoln, const vector<vector<double>> dMatrix, //const MSSoln* ms, /*vector<vector<Pt*>>& routes, */const int c,
    bool csv_print = false, bool print = false) {
    //const Problem* inst = ms->inst;
    // dMatrix includes launch/retrieve pts and free link back to launchpt
    //const vector<vector<double>> dMatrix = ms->clustSoln->clusters[c]->getdMatrix(c, make_pair(ms->launchPts[c], ms->launchPts[c + 1]));//[u];                        // for u vector in dMatrix
    vector<vector<Pt*>> routes = clustTendersoln.routes;
    //vector<vector<Pt*>> routes_new;
    //int n = ms->clustSoln->clusters.size();//centroids.size();  // number of clusters accounted for in main file...
    double gd_2opt_dists;
    int tenderCap = inst.tenderCap;
    if (print) cout << "\n---- GREEDY TENDER CLUSTERS ----\n";
    for (int v = 0; v < clustTendersoln.routes.size(); v++) {      // iterate for each tender in cluster
        vector<int> i_tour;                 //(tenderCap/*init_solution.mothership.centroidMatrix.size()*/, 0);                         // ai_tour = city_index for each tour -> nearest neighbour
        //printf("%d\t", v);
        for (auto &pt : routes[v]) {        //(int i = 0; i < tenderCap/*init_solution.mothership.centroidMatrix.size()*/; i++) {
            // return the index of stop in route list
            i_tour.push_back(findIndexByID(pt->ID, clustTendersoln.cluster.reefs, clustTendersoln.launchPts));              //ms->clustSoln->clusters/*init_solution.clustOrder.first*/[i];
        }//for(i=from_pts)

        pair<double, vector<int>> gd_out = gd_local_2opt_search(i_tour.size(), dMatrix, i_tour, false);      // args = (int ai_n, vector<vector<double>> &ad_dist, vector<int> &ai_tour, bool ab_full_nbrhd)
        gd_2opt_dists = gd_out.first;
        vector<int> gd_route_id = gd_out.second;
        double route_dist = clustTendersoln.getTenderRouteDist(v);        
        if (print) {
            printf("Vehicle\t\tInitial\t\tGreedy 2-Opt\n");
            printf("\t\t%7.3f  \t%7.3f\t", route_dist, gd_2opt_dists);
        }
        double improvement = route_dist - gd_2opt_dists;
        if (improvement > 0.0001 * route_dist) {                //if greedy solution is better than current
            if (print) printf("\t%.2f%%\tIMPROVEMENT\t", improvement * 100 / route_dist);
            route_dist = gd_2opt_dists;						 // update route dist
            vector<Pt*> route_new;
            vector<Pt*> reef_list = clustTendersoln.cluster.reefs;
            reef_list.insert(reef_list.begin(), clustTendersoln.launchPts.first);
            reef_list.push_back(clustTendersoln.launchPts.second);
            for (int a : gd_route_id) {    // for every stop INDEX in Gd route
                route_new.push_back(reef_list[a]);//getPtByID(gd_route_id[i], routes[v]));
		    }
            routes[v] = route_new;

            // vector<int> d = closeandOrientClusterLoop(gd_out, init_solution.mothership, init_solution.clustOrder);
            //init_solution.mothership.launch_stops = d;//UPDATE ROUTE LIST
            ////mothership.launch_stops = gd_out.second;//UPDATE ROUTE LIST
            //
            //init_solution.clustOrder = make_pair(init_solution.mothership.launch_stops, gd_out.first);
            //if (print) {
            //    cout << "\n\t\t";
            //    for (const auto& stop : init_solution.mothership.launch_stops) { cout << "\t" << stop; }
            //    cout << "\n\n";
            //}
        }
    }//for(v=vehicles)

    //if (print) {
    //    printMSroute(init_solution.mothership, centroids);
    //    cout << "------------- ^^Gd M/S^^ --------------\n";
    //}
    return routes;//*init_solution.clustOrder*/;
}
