#pragma once

//// RETURN reefs visited
//int reefsVisited(const vector<Pt*>& reefs) { 
//    int visit_count = 0;
//    vector<bool> visited(reefs.size(), false);
//    for (int i = 0; i < reefs.size(); i++) {
//        if (visited[i] == true) { visit_count++; }
//    }
//    return visit_count;
//}//reefsVisited

/// <summary>
/// 
/// </summary>
/// <param name="cluster"></param>
/// <param name="launchPts"></param>
/// <param name="printStops"></param>
/// <returns></returns>
vector<vector<Pt*>> TenderWithinClusterNearestNeighbour(//const MSSoln& ms, const int c,
    const ClusterSoln* cluster, const pair<Pt*, Pt*>& launchPts, bool printStops = false) {
    int u = -1;                                         // initialise current reef index
    vector <double> route_dists(inst.tenders.size(),0.0);// list of route dist for each vehicle
    //vector<vector<int>> route_stops;                  // create routes to output to csv
    if (printStops) { cout << "\n\nNEAREST NEIGHBOUR\n"; }
    vector<vector<Pt*>> routes(inst.tenders.size(), vector<Pt*>(inst.getTenderCap(), nullptr));  // create routes to save in TenderSoln.routes
    vector<bool> visited(cluster->reefs.size(), false);
    const vector<vector<double>> dMatrix = cluster->getdMatrix(launchPts);                  // for u vector in dMatrix

    for (int d = 0; d < inst.tenders.size(); d++) {     //FOR EACH VEHICLE!!                                    /*&& reefs_visit_count < cluster->reefs.size() - 2*/
        vector<Pt*> row;                            // create row for routes
        // add drop off pt to row
        double route_dist = 0;
        if (row.size() == 0) {                                          // if new vehicle = no reefs assigned to vehicle
            row.push_back(launchPts.first);                             // add launchPt to row
            u = 0;                                                      // set u as launchPt index
        }//if(v.stops==0)
        // Find the shortest trip from current node u to an unvisited node v
        for (int s = 0; s < inst.tenders[d].cap; s++) {                 // for stops within vehicle capacity    /* && reefs_visit_count < cluster->reefs.size()*/
            const vector<double>& neighbours = dMatrix[u];              // for u vector in dMatrix    //cluster->getdMatrix(d, make_pair(ms->launchPts[d], ms->launchPts[d]));//[u];
            double min = DBL_MAX;                                       // initialise min dist as MAX
            int v = -1;                                                 // initialise index as -1

            for (int w = 1; w < cluster->reefs.size()+1; w++) {         // for each reef in cluster
                if (neighbours[w] < min && visited[w-1] == false) {     // if closer unvisited reef:            /*&& cluster->reefs[w].id != -1 && cluster->reefs[w].id != -2*/
                    v = w;                                              // update index
                    min = neighbours[v];                                // update min
                }//if
            }//for

            route_dist += min;                              // add dist travelled to route dist
            u = v;                                                      // Update u=v as new index of closest pt
            visited[u-1] = true;                                    // Mark u=v(new) as visited.
            row.push_back(cluster->reefs[u-1]);
        }//for(d.capacity)
        // when one stop left, go to pick up pt
        int v = cluster->reefs.size() + 1;                  // careful here, v has already been used...
        route_dist += dMatrix[u][v];
        row.push_back(launchPts.second);

        if (printStops) {
            cout << "Tender: " << d << "\tStops:\t";
            for (int i = 0; i < row.size(); i++) { 
                //print reef ID and index in cluster->reefs
                printf("%d(%d)\t", findIndexByID(row[i]->ID, cluster->reefs, launchPts), row[i]->ID);
                //cout << row[i]->ID << "\t"; 
            }
            printf("\n\tDist:    \t%f\t", route_dist);
            //cout << "Return from final node: " << row.back()->ID << "\t(" << row.back()->x << ", " << row.back()->y << ")\n\t\t\t\t\t\tback to node: "; 
            printf("Return from final node: %d\t(%.2f, %.2f)\n\t\t\t\t\t\tback to node:\t", row.back()->ID, row.back()->x, row.back()->y);
        }
        row.push_back(row[0]);                  //append first stop to end of "row"
        if (printStops) { printf("%d\t(%.2f, %.2f)\tdone!\n", row.back()->ID, row.back()->x, row.back()->y); }
        // add distance of 2nd last->last stops to route_dist
        //route_dist += dMatrix.back()[0];      // this dist should be ZERO because it's from drop to pick nodes...

        routes[d] = row;
        route_dists[d] = route_dist;                  // append distance of vehicle route
        if (printStops) { cout << "Closed loop dist:\t" << route_dists[d] << "\n\n"; }
    }//for(vehicles)

    if (printStops) { cout << "Total vehicle route dist:\t" << sum(route_dists) << "\n"; }
    return routes;
}//nearestNeighbour

/// <summary>
/// returns vector of reef indices in cluster.reefs. 
/// note dMatrix includes launch/retrieve pts and free link back to launchpt
/// </summary>
/// <param name="clustTendersoln"></param>
/// <param name="dMatrix"></param>
/// <param name="print"></param>
/// <returns></returns>
vector<vector<Pt*>> greedyTenderCluster(const TenderSoln& clustTendersoln, const vector<vector<double>>& dMatrix, bool print = false) {
    ClusterSoln cluster = clustTendersoln.cluster;
    vector<vector<Pt*>> routes = clustTendersoln.routes;
    pair<Pt*, Pt*> launchPts = clustTendersoln.launchPts;
    double gd_2opt_dists;
    int tenderCap = inst.getTenderCap();
    if (print) printf("\n---- GREEDY TENDER CLUSTERS ----\n\tVehicle\t\tInitial\t\tGreedy 2-Opt\n");
    for (int d = 0; d < routes.size(); d++) {      // for each tender in cluster
        vector<int> i_tour;                 //(tenderCap/*init_solution.mothership.centroidMatrix.size()*/, 0);                         // ai_tour = city_index for each tour -> nearest neighbour
        for (auto& pt : routes[d]) {        //(int i = 0; i < tenderCap/*init_solution.mothership.centroidMatrix.size()*/; i++) {
            // return the index of every stop in route list
            i_tour.push_back(findIndexByID(pt->ID, cluster.reefs, launchPts));
        }//for(i=from_pts)

        pair<double, vector<int>> gd_out = gd_local_2opt_search(i_tour.size(), dMatrix, i_tour, true);      // args = (int ai_n, vector<vector<double>> &ad_dist, vector<int> &ai_tour, bool ab_full_nbrhd)
        gd_2opt_dists = gd_out.first;
        vector<int> gd_route_id = gd_out.second;
        double route_dist = clustTendersoln.getTenderRouteDist(d);
        if (print) { printf("\t%d\t\t%7.3f  \t%7.3f\t\n", d, route_dist, gd_2opt_dists > 999999 ? 999999 : gd_2opt_dists); } // if gd_2opt_dists is too large, print 9999}
        double improvement = route_dist - gd_2opt_dists;

        if (improvement > 0.0001 * route_dist) {                //if greedy solution is better than current
            if (print) { printf("\t%.2f%%\tIMPROVEMENT\t\n", improvement * 100 / route_dist); }
            vector<Pt*> route_new;
            route_new.push_back(launchPts.first);                       // add launchPt to route
            for (int i = 1; i < gd_route_id.size() - 2; i++) {          // for each stop in gd_route_id
                route_new.push_back(cluster.reefs[gd_route_id[i] - 1]); // add reef to route
            }
            route_new.push_back(launchPts.second);                      // add retrievePt to route
            route_new.push_back(launchPts.first);					    // return to launchPt
            routes[d] = route_new;									    // update route
        }
    }//for(d=vehicles)
    return routes;//*init_solution.clustOrder*/;
}

/// <summary>
/// returns vector of tenderSolns for each cluster: 
/// (Nearest Neighbour, then) Greedy 2-Opt added to tenderSolns
/// </summary>
/// <param name="msSoln"></param>
/// <param name="print"></param>
/// <returns></returns>
vector<TenderSoln> initTenderSoln(const MSSoln& msSoln, bool print=false) {   /*const vector<ClusterSoln*>& clusters, */
    vector<TenderSoln> tenderSolns;
    cout << string(30, '-') << "\n";
    printf("INITIALISE TENDER SOLUTIONS\n");
    cout << string(30, '-') << "\n";
    for (int c = 0; c < msSoln.clusters.size(); c++) {		// for each cluster
        const pair<Pt*, Pt*> launchPts = make_pair(msSoln.launchPts[c], msSoln.launchPts[c + 1]);		// launchPts for cluster
        ClusterSoln* cluster = msSoln.clusters[c];												// cluster
        if (print) {
            Pt centroid = cluster->getCentroid();	        // centroid for cluster	
            printf("\nCluster %d\tcentroid: (%.2f, %.2f)\n", cluster->ID, centroid.x, centroid.y);
        }
        // distance matrix for cluster
        vector<vector<double>> clusterMatrix = cluster->getdMatrix(launchPts);
        if (print) { for (int i = 0; i < clusterMatrix.size(); i++) { for (int j = 0; j < clusterMatrix[i].size(); j++) { { printf("%.2f\t", abs(clusterMatrix[i][j]) < DBL_MAX ? clusterMatrix[i][j] : 9999); } } printf("\n"); } printf("\n"); }
        //\\//\\//\\//\\// Initialise TenderSoln  //\\//\\//\\//\\//
        // Tendersoln Nearest Neighbour
        //vector<vector<Pt*>> tenderRoutes = TenderWithinClusterNearestNeighbour(msSoln, c);
        TenderSoln tenderSoln(*cluster,
            TenderWithinClusterNearestNeighbour(cluster, launchPts, print)/*tenderRoutes*/,
            launchPts);
        //\\//\\//\\//\\// TenderSoln Initialised //\\//\\//\\//\\//

        // Tendersoln Greedy 2-Opt update
        tenderSoln.routes = greedyTenderCluster(tenderSoln, clusterMatrix, print);
        tenderSolns.emplace_back(tenderSoln);
        // print routes
        //if (print) {
        printf("CLUSTER %d:\tID: %d\t\t\t\tCluster dist: %.2f\n", c, tenderSoln.cluster.ID, tenderSoln.getTenderRouteDist(-1));
        for (int i = 0; i < tenderSoln.routes.size(); i++) {
            printf("\t\t\tRoute %d:\n", i);
            for (int j = 0; j < tenderSoln.routes[i].size(); j++) {
                printf("\t\t\t\t%d\t(%.2f, %.2f)\n", tenderSoln.routes[i][j]->ID, tenderSoln.routes[i][j]->x, tenderSoln.routes[i][j]->y);
            }// for each node in route
        }// for each route
        cout << string(30, '-') << "\n";
        //}
    }// for each cluster
    return tenderSolns;
}
