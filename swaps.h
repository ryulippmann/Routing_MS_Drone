#pragma once
//#include <chrono>
//#include <ctime>
//#include <sstream>

#include <fstream>

#include "class_SA.h"

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

//Co-pilot function - update as needed
string csvPrintSA(SAlog log, SAparams sa_params, string file_name, string c) {
    string filename = file_name + "-" + c + ".csv";
    ofstream file(filename);
    if (file.is_open()) {
        file << "temp,current_dist,new_dist,best_dist\n";
        for (int i = 0; i < sa_params.num_iterations; i++) {
            file << log.temp[i] << "," << log.current_dist[i] << "," << log.new_dist[i] << "," << log.best_dist[i] << "\n";
        }
        file.close();
    }
    else printf("Unable to open file");
    return filename;
}

bool accept_new_solution(double current_distance, double new_distance, double temperature) {
    /*if (new_distance < current_distance) return true;
    else */return ((rand() / static_cast<double>(RAND_MAX)) < exp((current_distance - new_distance) / temperature));  // probability of accepting new solution (if worse)
}

/////////////////////////////////////////////////////////////////////////

// Function for swapping routes
// return pair of random indices [0, size_a) and [0, size_b) for swapping routes or stops
pair<int, int> randSwapChoice(int size_a, int randomSeed = 12345, int size_b = -1, bool within_clust = true) {       // RANDOMLY CHOOSE WHICH d_route TO SWAP
    // if size = 2, and routes must be different, set one to 0, one to 1
    if (size_b == -1) { size_b = size_a; }
    int d_route_a = getRandomNumber(size_a, randomSeed);
    int d_route_b = getRandomNumber(size_b, randomSeed%10);
    int count = 0;
    while (within_clust && d_route_a == d_route_b) {        // if same routes chosen
        d_route_b = getRandomNumber(size_b, randomSeed + count);    // re-choose route to swap out TO   current d_routes
        count++;
    }
    return make_pair(d_route_a, d_route_b);
}

// return random number [1, size-2] for swapping stops
int randChoice(int size, int randomSeed = 12345) {
    return getRandomNumber(size - 3, randomSeed) + 1;
}
//    mt19937 gen_stop(randomSeed);                      // check d_route to ensure d_route.size-2 is the right size distribution
//    uniform_int_distribution<int> dist(1, size - 3);     //!constrict swaps between launch/retrieval nodes - i.e. only reef nodes
//    //int from_stop = dist_from(gen_from_stop);   // d_route stop [1,10]    // choose stop number to swap out from current d_routes   
//    return dist(gen_stop);
//}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

// very similar to random_d_in_Swap... possibly duplicate or overload function
pair<TenderSoln, TenderSoln> random_d_out_Swap(pair<TenderSoln, TenderSoln> tenders/*const TenderSoln* tender_a, const TenderSotenders.first->ln* tender_b*/, int iteration,
    bool swap_print = false) { // d_route = ; tours = d_tours in this cluster

    /* RANDOMLY CHOOSE WHICH d_route TO SWAP */
    pair<int, int> s = randSwapChoice(  tenders.first.routes.size(), iteration, 
                                        tenders.second.routes.size(), false);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    /* RANDOMLY CHOOSE WHICH d_route TO SWAP FROM*/
    int from_stop = randChoice(tenders.first.routes[s.first].size(), iteration);        // choose stop number to swap out from current d_routes
    Pt* from_node = tenders.first.routes[s.first][from_stop];
    if (from_stop == 0 || from_stop > tenders.first.routes[s.first].size() - 3) { throw runtime_error("Error: Invalid from_stop chosen"); }
    // ^^ this only true if tender route contains launchpts - this is to safeguard them being swapped...
    
    /* RANDOMLY CHOOSE WHICH d_route TO SWAP TO*/
    int to_stop = randChoice(tenders.second.routes[s.second].size(), iteration);         // choose stop number to swap out from current d_routes
    Pt* to_node = tenders.second.routes[s.second][to_stop];
    if (to_stop == 0 || to_stop > tenders.second.routes[s.second].size() - 3) { throw runtime_error("Error: Invalid to_stop chosen"); }
    // ^^ this only true if tender route contains launchpts - this is to safeguard them being swapped...

    swap(tenders.first.cluster.reefs[findIndexByID(tenders.first.routes[s.first][from_stop]->ID, tenders.first.cluster.reefs)-1],
        tenders.second.cluster.reefs[findIndexByID(tenders.second.routes[s.second][to_stop]->ID, tenders.second.cluster.reefs)-1]);
    swap(tenders.first.routes[s.first][from_stop], tenders.second.routes[s.second][to_stop]);

    if (swap_print) {
        printf("\nSWAP:\td_routes\tindex\tnode\tReef_id\t\t(x,y)");
        printf("\nc\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.first, from_stop, from_node->ID, from_node->x, from_node->y);
        printf("\nd\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.second, to_stop, to_node->ID, to_node->x, to_node->y);
    }
    //swap(tenders.first->routes[s.first][from_stop], tenders.second->routes[s.second][to_stop]);

    //swap(tenders.first->cluster->reefs[findIndexByID(from_node->ID, tenders.first->cluster->reefs)], 
    //    tenders.second->cluster->reefs[findIndexByID(to_node->ID, tenders.first->cluster->reefs)]);
    return tenders;
}

// arg FullSoln incumbent -> changeable
FullSoln OUT_ClusterSwaps(const FullSoln soln, int iteration, bool print = false) {
    if (print) printf("---- OUT_Swap ----");
    pair<int, int> c = randSwapChoice(soln.msSoln.clusters.size(), iteration);    // generate swap pair of routes
    if (print) printf("\nSwap clusters:\t\t%d\tand\t%d", c.first, c.second);

    //throw runtime_error("TenderSoln created as pointer, need to be new object as clusters are swapped");
    pair<TenderSoln, TenderSoln>
        tenders = 
    //pair<ClusterSoln*, ClusterSoln*>
    //    clusters = make_pair(soln.msSoln->clusters[c.first], soln.msSoln->clusters[c.second]);
    random_d_out_Swap(make_pair(*soln.tenderSolns[c.first], *soln.tenderSolns[c.second]), iteration);

    pair <pair<Pt*, Pt*>, pair<Pt*, Pt*>>
        launchPts = make_pair(make_pair(soln.msSoln.launchPts[c.first], soln.msSoln.launchPts[c.first + 1]), make_pair(soln.msSoln.launchPts[c.second], soln.msSoln.launchPts[c.second + 1]));
    pair <vector<vector<double>>, vector<vector<double>>> 
        dMatrix = make_pair(tenders.first.cluster.getdMatrix(launchPts.first),
                            tenders.second.cluster.getdMatrix(launchPts.second));
    
    pair <vector<vector<Pt*>>, vector<vector<Pt*>>> routes;
    routes.first = greedyTenderCluster(tenders.first, dMatrix.first);
    routes.second = greedyTenderCluster(tenders.second, dMatrix.second);
    pair<vector<Pt*>, vector<Pt*>> reefs = make_pair(tenders.first.cluster.reefs, tenders.second.cluster.reefs);
    //create new clusterSoln* with updated reefs
    pair< ClusterSoln, ClusterSoln> clusters = make_pair(ClusterSoln(inst, reefs.first),
															ClusterSoln(inst, reefs.second));
    FullSoln new_soln = FullSoln(soln, routes, clusters, c);              // create new FullSoln copy incl new routes for updated clusters

    // DO THIS OUTSIDE THIS FUNCTION?!
    // RE-CLUSTER
    //pair<vector<vector<Reef_pt>>, vector<ClusterPoint> > cc = clusterAndCentroid(reefs, numClusters, noDrones, dCap, clusters, false, false);
    ////vector<vector<Reef_pt>> clusteredPoints = 
    //clusterAndCentroid(soln/*problem, solution*/, false, false);                                 //vector<ClusterPoint> centroids = cc.second;
    
    //for (int c = 0; c < clusteredPoints.size(); c++) {
    //    solution.clusters[c].reef_objects = clusteredPoints[c];
    //}
    //vector<pair<vector<vector<int>>, double>> drone_cluster_routes(problem.numClusters);
    // /////////////////////////////////////////////////////////////////
    /**** ADD LAUNCH AND RETRIEVAL NODES TO CLUSTERS ****/
    if (print) printf("-- ^^ OUT_Swap ^^ --");
    return new_soln;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

// SWAP TenderSoln routes directly by ref
void random_d_in_Swap(TenderSoln& tender, int iteration, bool swap_print = false) { // d_route = ; tours = d_tours in this cluster
    pair<int, int> s = randSwapChoice(tender.routes.size(), iteration);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    int from_stop = randChoice(tender.routes[s.first].size(), iteration);     // randomly choose stop number to swap out from current/chosen d_routes
    if (swap_print) printf("\nSwap From\tindex %d: node = %d\t", from_stop, tender.routes[s.first][from_stop]->ID);
    if (from_stop == 0 || from_stop > tender.routes[s.first].size()-3) {throw runtime_error ("Error: Invalid from_stop chosen");}

    int to_stop = randChoice(tender.routes[s.second].size(), iteration);      // randomly choose stop number to swap out from current d_routes
    if (swap_print) printf("\nTo\t\tindex %d: node = %d\t", to_stop, tender.routes[s.second][to_stop]->ID);
    if (to_stop == 0 || to_stop > tender.routes[s.second].size() - 3) { throw runtime_error("Error: Invalid to_stop chosen"); }

    swap(tender.routes[s.first][from_stop], tender.routes[s.second][to_stop]);

    if (swap_print) {
        for (const auto& d_route : tender.routes) {
            printf("\n\t");
            for (const auto& node : d_route) { cout << node << "\t"; }
        }
    } //cout << "\n";
    return;
}

// MODIFY Fullsoln soln (not ref) and return the swapped copy to save as proposed - mutated cluster_index c to swap within
FullSoln IN_ClusterSwaps(FullSoln soln, int iteration, bool print = false) {
    int c = randChoice(soln.msSoln.clusters.size(), iteration);    // generate random route to swap within
    if (print) {
        for (auto& route : soln.tenderSolns[c]->routes) {
            for (auto& node : route) {
                printf("\t%d ->", node->ID);
            }
            printf("\nRoute dist:\t%f\n", soln.tenderSolns[c]->getTenderRouteDist(route));
            //throw runtime_error("Error: Route dist calc not working!!");
        }
        cout << string(50, '~') << "\n";
    }
    // DONT CREATE COPY, as soln is NOT reference
    random_d_in_Swap(*soln.tenderSolns[c], iteration);
    if (print) {
        for (auto& route : soln.tenderSolns[c]->routes) {
            for (auto& node : route) {
                printf("\t%d ->", node->ID);
            }
            printf("\nRoute dist:\t%f\n", soln.tenderSolns[c]->getTenderRouteDist(route));
            //throw runtime_error("Error: Route dist calc not working!!");
        }
    }
    ClusterSoln* cluster = soln.msSoln.clusters[c];
    pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln.launchPts[c], soln.msSoln.launchPts[c + 1]);
    vector<vector<double>> dMatrix = cluster->getdMatrix(launchPts);
    vector<vector<Pt*>> routes = greedyTenderCluster(*soln.tenderSolns[c], dMatrix);
    FullSoln new_soln = FullSoln(soln, routes, c);
    /* PRINT ROUTES AND DISTS FOR EACH SUB - TOUR!! */
    if (print) {
        printf("\nOriginal route:\t%d\n", soln.getTotalDist());
        for (const auto& tender : new_soln.tenderSolns) {
            for (const auto& route : tender->routes) {
                for (const auto& node : route) {
                    printf("\t%d ->", node->ID);
                } printf("\n");
            } printf("\n");
        }
    }
    return new_soln;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

// Simulated Annealing based on in/out mutator
FullSoln SA_fn(const FullSoln initialSolution,
    function<FullSoln(const FullSoln currentSolution, const int, const bool)> mutator,
    const SAparams sa_params, SAlog& log, bool print = false) {
    FullSoln incumbent = initialSolution;
    FullSoln best = initialSolution;
    FullSoln proposed = initialSolution;
    double temp = sa_params.initial_temp;           // is this redundant? - temp is updated in SAlog
    for (int iter_num = 1; iter_num < sa_params.num_iterations; ++iter_num) {
        if (best.msSoln.launchPts.size() == 0) {
            throw runtime_error("Launch points not set!");
        }
        double dist_best = best.getTotalDist();
        double dist_incumbent = incumbent.getTotalDist();
        printf("\n%d\tbest: %.3f\tincumbent: %.3f",               iter_num, best.getTotalDist(), dist_incumbent);
        proposed = mutator(incumbent, iter_num, print);
        // WHY ARE LAUNCHPTS ERASED/RESET?! ... HERE ...
        //FullSoln =operator
        printf("stop here");
        dist_incumbent = incumbent.getTotalDist();
        dist_best = best.getTotalDist();
        double dist_proposed = proposed.getTotalDist();
        printf("\n%d\tbest: %.3f\tincumbent: %.3f\tproposed: %.3f", iter_num, best.getTotalDist(), dist_incumbent, dist_proposed);

        if (accept_new_solution(incumbent.getTotalDist(), proposed.getTotalDist(), temp)) {
            incumbent = proposed;       // overwrite old solution, but have been set as const...
            if (proposed.getTotalDist() < best.getTotalDist()) { best = proposed; }
        }
        temp *= sa_params.cooling_rate;
        log = SAlog(dist_proposed, dist_incumbent, dist_best, temp);        //update log
    }
    return best;
}

// Shell function setting up SA, and calling SA_fn with specified mutator (IN/OUT)
FullSoln SwapFunction(const FullSoln gd_soln, bool in_out, int num_iterations = 10000, double initial_temperature = 200, double cooling_rate = 0.999,
    bool print_stats = false, bool csv_print = false, bool SA_print = true) {   //in_out = 1; // 0 = OUT, 1 = IN
    if (in_out == 0) { printf("\n\nWithOUT Cluster\n"); }
    else if (in_out == 1) { printf("\n\nWithIN Cluster\n"); }
    else { throw runtime_error("Error: Invalid in_out argument provided"); }    
    printf("---------- CLUST_OPT_D_TOURS - Simulated Annealing ----------\n");
    SAlog log = SAlog(initial_temperature);

    FullSoln best(gd_soln);
    if (in_out == 0) {      //best = OUT_ClusterSwaps(gd_soln, 0, true);
        //SAparams(int num_iterations, double initial_temp, double cooling_rate)
        SAparams sa_params = SAparams(1000, 1500, 0.995);// initial temp ~ 2000 -> 
        best = SA_fn(gd_soln, OUT_ClusterSwaps, sa_params, log); 
    }
    else {                  //best = IN_ClusterSwaps(gd_soln, 0, true);
        SAparams sa_params = SAparams(400, 100, 0.99);
        best = SA_fn(gd_soln, IN_ClusterSwaps, sa_params, log);
    }
    //FullSoln best = FullSoln(gd_soln, OUT_ClusterSwaps, sa_params, log);

    //string filename_SA = "";
    //if (in_out == 0) {
    //    filename_SA = csvPrintSA(//log.solution_best_dist, log.solution_current_dist, log.solution_new_dist, log.solution_temp,
    //        //initial_temperature, cooling_rate, num_iterations, 
    //        log, sa_params, "d_opt_stops-SA_plot", "out");
    //}
    //else {
    //    filename_SA = csvPrintSA(//log.solution_best_dist, log.solution_current_dist, log.solution_new_dist, log.solution_temp,
    //        //initial_temperature, cooling_rate, num_iterations, 
    //        log, sa_params, "d_opt_stops-SA_plot", "in");
    //}
    cout << "\n------------- ^^ CLUST_OPT_D_TOURS ^^ --------------\n";
    return best;
}
