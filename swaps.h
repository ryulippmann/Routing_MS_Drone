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
    int d_route_b = getRandomNumber(size_b, randomSeed);
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

FullSoln OUT_ClusterSwaps(FullSoln soln, int iteration, bool print = false) {
    return soln;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

// SWAP TenderSoln routes directly by ref
void random_d_in_Swap(TenderSoln& tender, const vector<vector<double>> dMatrix, int iteration, bool swap_print = false) { // d_route = ; tours = d_tours in this cluster
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
    int c = randChoice(soln.msSoln->clusters.size(), iteration);    // generate random route to swap within
    ClusterSoln* cluster = soln.msSoln->clusters[c];
    pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln->launchPts[c], soln.msSoln->launchPts[c + 1]);
    vector<vector<double>> dMatrix = cluster->getdMatrix(launchPts);
    //pair<vector<vector<int>>, double> temp_route = random_d_in_Swap(soln.tenderSolns[c], clusters[c]->getdMatrix(launchPts), iteration); //, cluster.routes);
    
    // DONT CREATE COPY, as soln is NOT reference
    //TenderSoln* new_clust = soln.tenderSolns[c];
    //soln.tenderSolns[c]->routes = 
    random_d_in_Swap(*soln.tenderSolns[c], dMatrix, iteration);

    vector<vector<Pt*>> routes = greedyTenderCluster(soln.tenderSolns[c], dMatrix);//clusters[soln.clustOrder.first[c + 1] - 1]);       ///*vector<Reef_pt>& reefs, */
    // vector<vector<Pt*>> greedyTenderCluster(const TenderSoln* clustTendersoln, const vector<vector<double>> dMatrix
    //clusters[soln.clustOrder.first[c + 1] - 1] = new_clust;
    FullSoln new_soln = FullSoln(soln, routes, c);
    /* PRINT ROUTES AND DISTS FOR EACH SUB - TOUR!! */
    if (print) {
        printf("\nOriginal route:\t%d\n", soln.getTotalDist());
        for (const auto& tender : new_soln.tenderSolns) {
            for (const auto& route : tender->routes) {
                for (const auto& node : route) {
                    printf("\t%d ->", node->ID);
                } printf("\n");         //}printf("\t%d", node); 
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
    const SAparams sa_params, SAlog& log) {
    FullSoln incumbent = initialSolution;
    FullSoln best = initialSolution;
    double temp = sa_params.initial_temp;
    bool print = true;
    for (int iter_num = 1; iter_num < sa_params.num_iterations; ++iter_num) {
        FullSoln proposed = mutator(incumbent, iter_num, print);

        if (accept_new_solution(incumbent.getTotalDist(), proposed.getTotalDist(), temp)/*rand() / static_cast<double>(RAND_MAX) < exp((incumbent.getTotalDist() - proposed.getTotalDist()) / temperature)*/) {
            // overwrite old solution, but have been set as const...`            
            incumbent = proposed;//FullSoln(proposed.msSoln, proposed.tenderSolns);//FullSoln(proposed.clusters, proposed.mothership, proposed.routes, proposed.clustOrder);
            if (proposed.getTotalDist() < best.getTotalDist()) {
                best = proposed;//Solution(proposed.clusters, proposed.mothership, proposed.routes, proposed.clustOrder); //proposed;
            }
        }
        temp *= sa_params.cooling_rate;
    }
    //update log
    return best;
}

//string Swaps(FullSoln gd, bool in_out,
//    function<FullSoln(const FullSoln currentlSolution, const int, const bool)> mutator,
//    int num_iterations = 10000,
//    double initial_temperature = 200, double cooling_rate = 0.999,
//    bool print_stats = false, bool csv_print = false, bool SA_print = true) {
//    if (in_out == 0) { printf("\n\nWithOUT Cluster\n"); }
//    else if (in_out == 1) { printf("\n\nWithIN Cluster\n"); }
//    else { throw runtime_error("Error: Invalid in_out argument provided"); }
//    printf("---------- CLUST_OPT_D_TOURS - Simulated Annealing ----------\n");
//
//    // vector of results for SA plot
//    SAparams sa_params = SAparams(num_iterations, initial_temperature, cooling_rate);
//
//    SAlog log = SAlog(initial_temperature);
//
//    //FullSoln best = SA(gd, mutator, sa_params, log);
//    if (in_out == 0) { FullSoln best = SA(gd, OUT_ClusterSwaps, sa_params, log); }
//    else { FullSoln best = SA(gd, IN_ClusterSwaps, sa_params, log); }
//
//    string filename;
//
//    string filename_SA = "";
//
//    //if (SA_print) {
//    //    ////cout << "\nCurrent route:\t"    << log.current_dist.back() << "\n"; //cout << calculate_drone_distance(current_route, cluster.dMatrix) << "\t\t\t\t";        
//    //    ////for (const auto& cluster_routes : current_routes) {
//    //    ////    for (const auto& route : cluster_routes) {
//    //    ////        for (const auto& node : route) { cout << "\t" << node; } cout << "\n";
//    //    ////    } cout << "\n";
//    //    ////}        
//    //    cout << "Best route:\t" << log.best_dist.back() << "\n";
//    //    //for (const auto& tender : best.tenderSolns) {
//    //    //    for (const auto& route : tender->routes) {
//    //    //        for (const auto& node : route) { 
//    //    //            cout << "\t" << node; } 
//    //    //        cout << "\n";
//    //    //    } 
//    //    //    cout << "\n";
//    //    //}
//    //
//    //    if (in_out == 0) {
//    //        filename_SA = csvPrintSA(//log.solution_best_dist, log.solution_current_dist, log.solution_new_dist, log.solution_temp,
//    //            //initial_temperature, cooling_rate, num_iterations, 
//    //            log, sa_params, "d_opt_stops-SA_plot", "out");
//    //    }
//    //    else {
//    //        filename_SA = csvPrintSA(//log.solution_best_dist, log.solution_current_dist, log.solution_new_dist, log.solution_temp,
//    //            //initial_temperature, cooling_rate, num_iterations, 
//    //            log, sa_params, "d_opt_stops-SA_plot", "in");
//    //    }
//    //}
//
//    return filename;
//}

// Shell function setting up SA, and calling SA_fn with specified mutator (IN/OUT)
string SwapFunction(const FullSoln gd_soln, bool in_out, int num_iterations = 10000, double initial_temperature = 200, double cooling_rate = 0.999,
    bool print_stats = false, bool csv_print = false, bool SA_print = true) {   //in_out = 1; // 0 = OUT, 1 = IN
    if (in_out == 0) { printf("\n\nWithOUT Cluster\n"); }
    else if (in_out == 1) { printf("\n\nWithIN Cluster\n"); }
    else { throw runtime_error("Error: Invalid in_out argument provided"); }
    
    printf("---------- CLUST_OPT_D_TOURS - Simulated Annealing ----------\n");

    // vector of results for SA plot
    SAparams sa_params = SAparams(num_iterations, initial_temperature, cooling_rate);
    SAlog log = SAlog(initial_temperature);

    FullSoln best(gd_soln);// = gd_soln;    
    if (in_out == 0) {      //best = OUT_ClusterSwaps(gd_soln, 0, true);
        best = SA_fn(gd_soln, OUT_ClusterSwaps, sa_params, log); 
    }
    else {                  //best = IN_ClusterSwaps(gd_soln, 0, true);
        best = SA_fn(gd_soln, IN_ClusterSwaps, sa_params, log);
    }
    //FullSoln best = FullSoln(gd_soln, OUT_ClusterSwaps, sa_params, log);

    //clusters = best_clusters;
    string filename_SA = "";
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
    return filename_SA; /*routes*/
}//    double gd_2opt_dist;
