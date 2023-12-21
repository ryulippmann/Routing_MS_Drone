#pragma once

//vector<vector<double>> calc_centMatrix(pair<double, double> depot, vector <ClusterPoint> centroids, bool print = false) {
//    if (centroids[0].x != depot.first && centroids[0].y != depot.second) {
//        centroids.insert(centroids.begin(), ClusterPoint(depot.first, depot.second));
//    }
//    vector<vector<double>> centMatrix(centroids.size(), vector<double>(centroids.size(), 0));
//    //vector<double> depotDist;
//    //depotDist.push_back(0);
//    //for (const ClusterPoint& c : centroids) {
//    //    depotDist.push_back(sqrt(pow((depot.first - c.x),2) + pow((depot.second - c.y),2)));
//    //}
//    for (int i = 0; i < centroids.size(); i++) {
//        for (int j = 0; j < centroids.size(); j++) {
//            centMatrix[i][j] = (sqrt(pow((centroids[i].x - centroids[j].x), 2) + pow(centroids[i].y - centroids[j].y, 2)));
//        }
//    }
//    //centMatrix.insert(centMatrix.begin(), depotDist);
//    //for (int a = 1; a < centMatrix.size(); a++) { centMatrix[a].insert(centMatrix[a].begin(), depotDist[a]); }
//
//    if (print) {
//        printf("\nFrom depot to cluster centroids:\n");     // print centroidMatrix
//        for (int i = 0; i < centroids.size(); i++) printf("\t%d", i);
//        printf("\n");
//        int j = 0;
//        for (const vector<double>& line : centMatrix) { printf("%d", j); for (const double& dist : line) { printf("\t%2.2f", dist); } cout << "\n"; j++; }
//    }
//    return centMatrix;
//}

//pair<vector<pair<double, double>>, double> setLaunchPts(const vector<int> clusterOrder, vector<ClusterPoint> centroids, const pair<double, double> depot, bool print = false) {
//    vector<pair<double, double>> launchpts;
//    launchpts.push_back(depot);
//    centroids.insert(centroids.begin(), ClusterPoint(depot.first, depot.second));
//    // using clusterOrder,
//    for (int c = 0; c < clusterOrder.size() - 1; c++) {
//        pair<double, double> launchpt = make_pair( // sum adjacent clusters x,y's to calc launchpts and 
//            (centroids[clusterOrder[c]].x + centroids[clusterOrder[c + 1]].x) / 2,
//            (centroids[clusterOrder[c]].y + centroids[clusterOrder[c + 1]].y) / 2);
//        if (print) printf("\n%d -> %d\t(%2.2f, %2.2f)", clusterOrder[c], clusterOrder[c + 1], launchpt.first, launchpt.second);
//        launchpts.push_back(launchpt); // store these coords in launchpts
//    }
//    //pair<double, double> launchpt = make_pair( // sum adjacent clusters x,y's to calc launchpts and 
//    //    (centroids[clusterOrder[clusterOrder.size() - 1]].x + centroids[clusterOrder[0]].x) / 2,
//    //    (centroids[clusterOrder[clusterOrder.size() - 1]].y + centroids[clusterOrder[0]].y) / 2);
//    //if (print) printf("\nclusters here are one index too large.\nHere depot = 0");
//    ////printf("\n%d -> %d\t(%2.2f, %2.2f)", clusterOrder[clusterOrder.size() - 1], clusterOrder[0], launchpt.first, launchpt.second);
//    //launchpts.push_back(launchpt); // store these coords in launchpts
//
//    //launchpts.push_back(depot);       // Add this line if Greedy function NOT run
//    double route_dist = 0;
//    for (int s = 0; s < launchpts.size() - 1; s++) {
//        route_dist += calcPtDist(launchpts[s], launchpts[s + 1]);
//    }
//    pair<vector<pair<double, double>>, double> ms_launchpt_route = make_pair(launchpts, route_dist);
//    return ms_launchpt_route;
//}

//void clusterAndCentroid(FullSoln& soln,
//    //vector<Cluster>& clusters, /*const vector<Reef_pt>& reefs, int numClusters, const int noDrones, const int dCap,
//    //vector<vector<Reef_pt>>& clusteredPoints, vector<ClusterPoint>& centroids,*/  
//    bool clusterPrint = false, bool csvPrint = false) {
//    if (clusterPrint) printf("\n------------ clusterAndCentroid ------------\n");
//    //vector<Cluster> clusters = init_solution.clusters;
//    vector<vector<Pt*>> clusteredPoints;
//    vector<ClusterSoln*> clusters = soln.msSoln->clusters;
//    Problem problem = soln.msSoln->inst;
//    if (clusters.empty()) { clusters = kMeansConstrained(problem); }//.reefs, problem.numClusters, clusterPrint, csvPrint/*, randomSeed, maxIterations*/); }   // only happens at first initialisation
//    else { for (const auto& cluster : clusters) { clusteredPoints.push_back(cluster->reefs); } }       // else: use existing clusters (don't redo kMeans)
//    ///* consider updating to multiple instances of calcCentroid function to avoid duplicate functions */    
//    //vector<ClusterPoint> centroids = getClusterCentroids(clusteredPoints, clusterPrint);
//    //initClusters(clusters, problem.numClusters, createVehicles(problem.noDrones, problem.dCap), clusteredPoints, centroids);
//    //// create vector of depot, centroids (, depot) //vector<int> 
//    //pair<double, double> depot = make_pair(0.0, 0.0);//{ -1, 0, 0 };       // make depot node (-1) at origin       //1;
//    //init_solution.mothership.centroidMatrix = calc_centMatrix(depot, centroids);   // initialise centMatrix
//    ////ORDER CLUSTERS: CENTROID ROUTING - NEAREST NEIGHBOUR
//    //init_solution.clustOrder = clusterCentroidNearestNeighbour(problem, init_solution, clusterPrint, csvPrint);
//    ////centroids, problem.mothership, centroidMatrix, csvPrint, false);
//    ////ORDER CLUSTERS: CENTROID ROUTING - GREEDY 2-OPT
//    ///*clustOrder = */greedyCluster(init_solution, /*problem.mothership, centroidMatrix, clustOrder*/centroids);  //CALL GREEDY
//    //// Route mothership from depot to !LAUNCH PTS! (for centroids), and back
//    //pair<vector<pair<double, double>>, double> ms_launchpt_route = setLaunchPts(init_solution.clustOrder.first, centroids, depot, true);
//    //for (int c = 1; c < clusters.size() + 1; c++) {                                 // for each cluster (in sequential order of visiting)
//    //    clusters[init_solution.clustOrder.first[c] - 1].start = ms_launchpt_route.first[c];       // add launchpt to clusters.start
//    //    clusters[init_solution.clustOrder.first[c] - 1].stop = ms_launchpt_route.first[c + 1];    // add launchpt to clusters.stop
//    //}
//    //init_solution.mothership.route.first = ms_launchpt_route.first;
//    //init_solution.mothership.route_dist = ms_launchpt_route.second;
//    //for (int c = 1; c < init_solution.mothership.launch_stops.size() - 1; c++) {    // for each cluster number = c (not including depot at beginning and end)
//    //    if (init_solution.clustOrder.first[c] == 0) {
//    //        throw runtime_error("Zero node passed to writeClusterdMatrix");
//    //    }
//    //    else {       // not depot
//    //        if (clusterPrint) { printf("\nCluster %d:\t(%2.2f, %2.2f)\t->\t(%2.2f, %2.2f)\n", init_solution.clustOrder.first[c] - 1, ms_launchpt_route.first[c].first, ms_launchpt_route.first[c].second, ms_launchpt_route.first[c + 1].first, ms_launchpt_route.first[c + 1].second); }
//    //        writeClusterdMatrix(init_solution/*.clusters[init_solution.clustOrder.first[c] - 1], clusteredPoints[init_solution.clustOrder.first[c] - 1], ms_launchpt_route.first[c], ms_launchpt_route.first[c + 1]*/, c, clusterPrint);
//    //    }
//    //}
//    //for (int i = 0; i < clusteredPoints.size(); i++) {
//    //    clusters[i].reef_objects = clusteredPoints[i];
//    //    clusters[i].centroid = centroids[i];
//    //}
//    if (clusterPrint) cout << "\n------------------ ^^ NO clust&cent NO ^^ ------------------\n";
//    return /*clusteredPoints;*//*make_pair(clusteredPoints, centroids)*/;
//}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

////pair<vector<vector<int>>, double> = route, dist?
//vector<vector<Pt*>> random_d_in_Swap(TenderSoln* tender, const vector<vector<double>> dMatrix, int iteration, bool swap_print = false) { // d_route = ; tours = d_tours in this cluster
//    // check if tender.routes.size() > 2
//    /* RANDOMLY CHOOSE WHICH d_route TO SWAP*/
//    pair<int, int> s = randSwapChoice(tender->routes.size(), iteration);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
//    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);
//
//    /* RANDOMLY CHOOSE WHICH STOP IN CHOSEN d_routes TO SWAP FROM*/
//    int from_stop = randChoice(tender->routes[s.first].size(), iteration);     // choose stop number to swap out from current d_routes
//    //dist_from(gen_from_stop);   // d_route stop [1,10]    // choose stop number to swap out from current d_routes
//    if (swap_print) printf("\nSwap From\tindex %d: node = %d\t", from_stop, tender->routes[s.first][from_stop]);
//
//    /* RANDOMLY CHOOSE WHICH STOP IN CHOSEN d_routes TO SWAP TO*/
//    int to_stop = randChoice(tender->routes[s.second].size(), iteration);      // choose stop number to swap out from current d_routes
//    //dist_to(gen_to_stop);   // d_route stop [1,10]    // choose stop number to swap out from current d_routes
//    if (swap_print) printf("\nTo\t\tindex %d: node = %d\t", to_stop, tender->routes[s.second][to_stop]);
//
//    swap(tender->routes[s.first][from_stop], tender->routes[s.second][to_stop]);
//
//    if (swap_print) {
//        for (const auto& d_route : tender->routes) {
//            printf("\n\t");
//            for (const auto& node : d_route) { cout << node << "\t"; }
//        }
//    } //cout << "\n";
//    return tender->routes;//make_pair(d_routes, (d_routes, dMatrix));
//}

//FullSoln IN_ClusterSwaps(FullSoln soln, int iteration, bool print = false) {
//    vector<ClusterSoln*> clusters = soln.msSoln->clusters;
//    int c = randSwapChoice(clusters.size(), iteration).first;    // generate swap pair of routes
//    pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln->launchPts[c], soln.msSoln->launchPts[c + 1]);
//    //pair<vector<vector<int>>, double> temp_route = random_d_in_Swap(soln.tenderSolns[c], clusters[c]->getdMatrix(launchPts), iteration); //, cluster.routes);
//    //vector<vector<Pt*>> abc =   random_d_in_Swap(soln.tenderSolns[c], clusters[c]->getdMatrix(launchPts), iteration);//, bool swap_print = false)
//    TenderSoln new_clust(clusters[c], random_d_in_Swap(soln.tenderSolns[c], clusters[c]->getdMatrix(launchPts), iteration), launchPts);            //drone_cluster_routes[c] =
//    // transform temp_route.first to vector<vector<Pt*>> routes
//
//    //for (int d = 0; d < soln.tenderSolns[c].routes.size(); d++) {   // UPDATE ROUTES incl -1 and -2 nodes!
//    //    new_clust.routes[d].push_back(0);
//    //    for (int s = 0; s < soln.tenderSolns[c].routes[d].size(); s++) {
//    //        new_clust.routes[d].push_back(clusters[c]->reefs[soln.tenderSolns[c].routes[d][s] - 1]);//new_clust.routes[d].reef_stops.push_back(new_cluster.routes[d][s]);
//    //    }
//    //}
//
//    //new_clust.routes = temp_route.first;        // vector<vector<Pt*>> <- vector<vector<int>> routes
//    //new_clust.route_dist = temp_route.second;
//    new_clust.routes = greedyTenderCluster(soln.tenderSolns[c], soln.tenderSolns[c]->cluster->getdMatrix(launchPts));//clusters[soln.clustOrder.first[c + 1] - 1]);       ///*vector<Reef_pt>& reefs, */
//    // vector<vector<Pt*>> greedyTenderCluster(const TenderSoln* clustTendersoln, const vector<vector<double>> dMatrix
//    //clusters[soln.clustOrder.first[c + 1] - 1] = new_clust;
//
//    /* PRINT ROUTES AND DISTS FOR EACH SUB - TOUR!! */
//    if (print) {
//        printf("\nOriginal route:\t%d\n", soln.getTotalDist());
//        for (const auto& tender : soln.tenderSolns) {
//            for (const auto& route : tender->routes) {
//                for (const auto& node : route) {
//                    printf("\t%d ->", node->ID);
//                } printf("\n");         //}printf("\t%d", node); 
//            } printf("\n");
//        }
//    }
//    return soln;
//}

/////////////////////////////////////////////////////////////////////////

//void random_d_out_Swap(const TenderSoln* tender_a, const TenderSoln* tender_b, int iteration,
//    bool swap_print = false) { // d_route = ; tours = d_tours in this cluster
//
//    /* RANDOMLY CHOOSE WHICH d_route TO SWAP */
//    pair<int, int> s = randSwapChoice(tender_a->routes.size(), iteration, tender_b->routes.size(), false);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
//
//    /* RANDOMLY CHOOSE WHICH d_route TO SWAP FROM*/
//    int from_stop = randChoice(tender_a->routes[s.first].size(), iteration);        // choose stop number to swap out from current d_routes
//    Pt* from_node = tender_a->routes[s.first][from_stop];
//
//    /* RANDOMLY CHOOSE WHICH d_route TO SWAP TO*/
//    int to_stop = randChoice(tender_b->routes[s.second].size(), iteration);         // choose stop number to swap out from current d_routes
//    Pt* to_node = tender_b->routes[s.second][to_stop];
//    if (swap_print) {
//        printf("\nSWAP:\td_routes\tindex\tnode\tReef_id\t\t(x,y)");
//        printf("\nc\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.first, from_stop, from_node->ID, from_node->x, from_node->y);
//        printf("\nd\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.second, to_stop, to_node->ID, to_node->x, to_node->y);
//    }
//    //swap(tender_a->routes[s.first][from_stop], tender_b->routes[s.second][to_stop]);
//
//    //swap(tender_a->cluster->reefs[findIndexByID(from_node->ID, tender_a->cluster->reefs)], 
//    //    tender_b->cluster->reefs[findIndexByID(to_node->ID, tender_a->cluster->reefs)]);
//    return;
//}

//FullSoln OUT_ClusterSwaps(FullSoln soln, int iteration, bool print = false) {
//    if (print) printf("---- OUT_Swap ----");
//    vector<ClusterSoln*> clusters = soln.msSoln->clusters;
//    pair<int, int> c = randSwapChoice(clusters.size(), iteration);    // generate swap pair of routes
//    if (print) printf("\nSwap clusters:\t\t%d\tand\t%d", c.first, c.second);
//    random_d_out_Swap(soln.tenderSolns[c.first], soln.tenderSolns[c.second], iteration);
//
//    // DO THIS OUTSIDE THIS FUNCTION?!
//    // RE-CLUSTER
//    //pair<vector<vector<Reef_pt>>, vector<ClusterPoint> > cc = clusterAndCentroid(reefs, numClusters, noDrones, dCap, clusters, false, false);
//    /*vector<vector<Reef_pt>> clusteredPoints = */clusterAndCentroid(soln/*problem, solution*/, false, false);                                 //vector<ClusterPoint> centroids = cc.second;
//    //for (int c = 0; c < clusteredPoints.size(); c++) {
//    //    solution.clusters[c].reef_objects = clusteredPoints[c];
//    //}
//    //vector<pair<vector<vector<int>>, double>> drone_cluster_routes(problem.numClusters);
//    // /////////////////////////////////////////////////////////////////
//    /**** ADD LAUNCH AND RETRIEVAL NODES TO CLUSTERS ****/
//    for (int a = 1; a < clusters.size() + 1; a++) {
//        pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln->launchPts[a], soln.msSoln->launchPts[a + 1]);
//        ////Add drop off and pick up pts to each cluster...
//        TenderSoln new_clustTendersoln(clusters[a], TenderWithinClusterNearestNeighbour(soln.msSoln, a),
//            launchPts);		//vector<vector<Pt*>> cluster_routes = TenderWithinClusterNearestNeighbour(&msSoln, c);		
//        //Cluster* new_cluster;
//        //new_cluster = TenderWithinClusterNearestNeighbour(        // INITIALISE SOLUTION = Nearest Neighbour for each cluster...
//        //    clusters[solution.clustOrder.first[a] - 1]);
//        for (int d = 0; d < new_clustTendersoln.routes.size(); d++) {   // UPDATE ROUTES incl -1 and -2 nodes!
//            //new_cluster.drones[d].reef_stops.push_back(0/*-1*/);
//            for (int s = 0; s < new_clustTendersoln.routes[d].size(); s++) {
//                ;//new_clustTendersoln.routes[d].reef_stops.push_back(new_cluster.routes[d][s]);
//            }
//        }
//        new_clustTendersoln.routes = greedyTenderCluster(&new_clustTendersoln, clusters[a]->getdMatrix(launchPts));     // IMPROVE SOLUTION = Greedy 2-Opt for each cluster...
//        //Cluster* cluster, vector<vector<Pt*>> routes, pair<Pt*, Pt*> launchPts
//        //soln.tenderSolns[a] = TenderSoln(new_clustTendersoln;
//    }
//
//    if (print) printf("-- ^^ OUT_Swap ^^ --");
//    return soln;
//}

//FullSoln IN_ClusterSwaps(FullSoln soln, int iteration, bool print = false) {
//    int c = randChoice(soln.msSoln->clusters.size(), iteration);    // generate random route to swap within
//    //pair<vector<vector<int>>, double> temp_route = random_d_in_Swap(soln.tenderSolns[c], clusters[c]->getdMatrix(launchPts), iteration); //, cluster.routes);
//
//    // DONT CREATE COPY, as soln is NOT reference
//    //TenderSoln* new_clust = soln.tenderSolns[c];
//    //soln.tenderSolns[c]->routes = 
//    random_d_in_Swap(*soln.tenderSolns[c], iteration);
//
//    ClusterSoln* cluster = soln.msSoln->clusters[c];
//    pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln->launchPts[c], soln.msSoln->launchPts[c + 1]);
//    vector<vector<double>> dMatrix = cluster->getdMatrix(launchPts);
//
//    vector<vector<Pt*>> routes = greedyTenderCluster(soln.tenderSolns[c], dMatrix);//clusters[soln.clustOrder.first[c + 1] - 1]);       ///*vector<Reef_pt>& reefs, */
//    // vector<vector<Pt*>> greedyTenderCluster(const TenderSoln* clustTendersoln, const vector<vector<double>> dMatrix
//    //clusters[soln.clustOrder.first[c + 1] - 1] = new_clust;
//    FullSoln new_soln = FullSoln(soln, routes, c);
//    /* PRINT ROUTES AND DISTS FOR EACH SUB - TOUR!! */
//    if (print) {
//        printf("\nOriginal route:\t%d\n", soln.getTotalDist());
//        for (const auto& tender : new_soln.tenderSolns) {
//            for (const auto& route : tender->routes) {
//                for (const auto& node : route) {
//                    printf("\t%d ->", node->ID);
//                } printf("\n");         //}printf("\t%d", node); 
//            } printf("\n");
//        }
//    }
//    return new_soln;
//}

