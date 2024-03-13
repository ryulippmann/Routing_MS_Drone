#pragma once
#include <fstream>

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////


bool accept_new_solution(double current_dist, double proposed_dist, double temperature) {
    /*if (proposed_dist < current_dist) return true;
    else */return ((rand() / static_cast<double>(RAND_MAX)) < exp((current_dist - proposed_dist) / temperature));  // probability of accepting new solution (if worse)
}

/////////////////////////////////////////////////////////////////////////

// Function for swapping routes
// return pair of random indices [0, size_a) and [0, size_b) for swapping routes or stops
pair<int, int> randSwapChoice(int size_a, /*int randomSeed = 12345, */int size_b = -1, bool duplicate_rule = true) {       // RANDOMLY CHOOSE WHICH d_route TO SWAP
    // if size = 2, and routes must be different, set one to 0, one to 1
    if (size_b == -1) { size_b = size_a; }
    int d_route_a = getRandomNumber(size_a/*, randomSeed*/);
    int d_route_b = getRandomNumber(size_b/*, randomSeed%10*/);
    int count = 0;
    while (duplicate_rule && d_route_a == d_route_b) {        // if same routes chosen
        d_route_b = getRandomNumber(size_b/*, randomSeed + count*/);    // re-choose route to swap out TO   current d_routes
        count++;
    }
    return make_pair(d_route_a, d_route_b);
}

// return random number [1, size-2] for swapping stops
int randChoice(int size/*, int randomSeed = 12345*/, bool clust_choice = false) {
    if (clust_choice) return getRandomNumber(size);
    else return getRandomNumber(size - 3/*, randomSeed*/) + 1;
}
//    mt19937 gen_stop(randomSeed);                      // check d_route to ensure d_route.size-2 is the right size distribution
//    uniform_int_distribution<int> dist(1, size - 3);     //!constrict swaps between launch/retrieval nodes - i.e. only reef nodes
//    //int from_stop = dist_from(gen_from_stop);   // d_route stop [1,10]    // choose stop number to swap out from current d_routes   
//    return dist(gen_stop);
//}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

// very similar to random_d_in_Swap... possibly duplicate or overload function
pair<TenderSoln, TenderSoln> random_d_out_Swap(pair<TenderSoln, TenderSoln> tenders, int iteration,
    bool swap_print = false) { // d_route = ; tours = d_tours in this cluster

    /* RANDOMLY CHOOSE WHICH d_route TO SWAP */
    pair<int, int> s = randSwapChoice(  tenders.first.routes.size(), //iteration, 
                                        tenders.second.routes.size(), false);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    // pair<int, int> stops = randSwapChoice(..., ..., false); // make sure indices are correct. i.e. +1 ?

    /* RANDOMLY CHOOSE WHICH d_route TO SWAP FROM*/
    int from_stop = randChoice(tenders.first.routes[s.first].size()/*, iteration*/);        // choose stop number to swap out from current d_routes
    Pt* from_node = tenders.first.routes[s.first][from_stop];
    if (from_stop == 0 || from_stop > tenders.first.routes[s.first].size() - 3) { throw runtime_error("Error: Invalid from_stop chosen"); }
    // ^^ this only true if tender route contains launchpts - this is to safeguard them being swapped...
    
    /* RANDOMLY CHOOSE WHICH d_route TO SWAP TO*/
    int to_stop = randChoice(tenders.second.routes[s.second].size()/*, iteration*/);         // choose stop number to swap out from current d_routes
    Pt* to_node = tenders.second.routes[s.second][to_stop];
    if (to_stop == 0 || to_stop > tenders.second.routes[s.second].size() - 3) { throw runtime_error("Error: Invalid to_stop chosen"); }
    // ^^ this only true if tender route contains launchpts - this is to safeguard them being swapped...

    swap(tenders.first.cluster.reefs[findIndexByID(tenders.first.routes[s.first][from_stop]->ID, tenders.first.cluster.reefs)-1],
        tenders.second.cluster.reefs[findIndexByID(tenders.second.routes[s.second][to_stop]->ID, tenders.second.cluster.reefs)-1]);
    swap(tenders.first.routes[s.first][from_stop], tenders.second.routes[s.second][to_stop]);

    if (swap_print) {
        printf("\nSWAP:\td_routes\tindex\tnode\tReef_id\t\t(x,y)");
        printf("\nc\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.first,   from_stop,  from_node->ID,  from_node->x,   from_node->y);
        printf("\nd\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.second,  to_stop,    to_node->ID,    to_node->x,     to_node->y);
    }
    return tenders;
}

// arg FullSoln incumbent
FullSoln OUT_ClusterSwaps(const FullSoln& soln, int iteration, vector<int> randoms, bool print = false) {
    if (print) printf("---- OUT_Swap ----");
    pair<int, int> c = randSwapChoice(soln.msSoln.clusters.size()/*, iteration*/);    // generate swap pair of clusters
    if (print) printf("\nSwap clusters:\t\t%d\tand\t%d", c.first, c.second);

    //pair<ClusterSoln*, ClusterSoln*> clusters = make_pair(soln.msSoln->clusters[c.first], soln.msSoln->clusters[c.second]);
    pair<TenderSoln, TenderSoln>
        tenders = random_d_out_Swap(make_pair(*soln.tenderSolns[c.first], *soln.tenderSolns[c.second]), iteration);
    
    pair <pair<Pt*, Pt*>, pair<Pt*, Pt*>>
        launchPts = make_pair(make_pair(soln.msSoln.launchPts[c.first], soln.msSoln.launchPts[c.first + 1]), make_pair(soln.msSoln.launchPts[c.second], soln.msSoln.launchPts[c.second + 1]));
    pair <vector<vector<double>>, vector<vector<double>>> 
        dMatrix = make_pair(tenders.first.cluster.getdMatrix(launchPts.first),
                            tenders.second.cluster.getdMatrix(launchPts.second));
    
    pair <vector<vector<Pt*>>, vector<vector<Pt*>>> routes;
    routes.first = greedyTenderCluster(tenders.first, dMatrix.first);   		                // Run greedy 2-Opt on routes      
    routes.second = greedyTenderCluster(tenders.second, dMatrix.second);		                // Run greedy 2-Opt on routes 

    pair<vector<Pt*>, vector<Pt*>> reefs = make_pair(tenders.first.cluster.reefs, tenders.second.cluster.reefs);
    //create new clusterSoln* with updated reefs
    pair< ClusterSoln, ClusterSoln> clusters = make_pair(ClusterSoln(inst, reefs.first),
															ClusterSoln(inst, reefs.second));
    FullSoln new_soln = FullSoln(soln, routes, clusters, c);              // create new FullSoln copy incl new routes for updated clusters

    if (print) printf("-- ^^ OUT_Swap ^^ --");
    return new_soln;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

// SWAP TenderSoln routes directly by ref
void random_d_in_Swap(TenderSoln& tender, /*int iteration, */bool swap_print = false) { // d_route = ; tours = d_tours in this cluster
    pair<int, int> s = randSwapChoice(tender.routes.size()/*, iteration*/);   // RANDOMLY CHOOSE WHICH d_routes TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    // pair<int, int> stops = randSwapChoice(..., ..., false); // make sure indices are correct. i.e. +1 ?

    int from_stop = randChoice(tender.routes[s.first].size()/*, iteration*/);     // randomly choose stop number to swap out from current/chosen d_routes
    if (swap_print) printf("\nSwap From\tindex %d: node = %d\t", from_stop, tender.routes[s.first][from_stop]->ID);
    if (from_stop == 0 || from_stop > tender.routes[s.first].size()-3) {throw runtime_error ("Error: Invalid from_stop chosen");}

    int to_stop = randChoice(tender.routes[s.second].size()/*, iteration*/);      // randomly choose stop number to swap out from current d_routes
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

//// print tender routes and dists - based on tenderSoln* tender
//void printTenderRoutes(const TenderSoln* tender) {
//	for (const auto& route : tender->routes) {
//        for (auto& node : route) {
//            printf("\t%d ->", node->ID);
//        }
//        printf("\nRoute dist:\t%f\n", tender->getTenderRouteDist(route));
//    }
//}

// MODIFY Fullsoln soln (not ref) and return the swapped copy to save as proposed - mutated cluster_index c to swap within
FullSoln IN_ClusterSwaps(FullSoln soln, int iteration, vector<int> randoms, bool print = false) {
    int c = randChoice(soln.msSoln.clusters.size()/*, iteration*/, true);    // generate random CLUSTER to swap within
    //throw runtime_error("Not all clusters swappable! randChoice limits choice [1,clusters.size()-3]");
    if (print) {
        cout << "\n" << string(50, '~') << "\n";
        printf("\nSwap cluster:\t\t%d\n", c);
        //printTenderRoutes(soln.tenderSolns[c]);
    }
    // DONT CREATE COPY, as soln is NOT reference
    random_d_in_Swap(*soln.tenderSolns[c]/*, iteration*/);
    //if (print) {
    //    printTenderRoutes(soln.tenderSolns[c]);
    //}
    ClusterSoln* cluster = soln.msSoln.clusters[c];
    pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln.launchPts[c], soln.msSoln.launchPts[c + 1]);
    vector<vector<double>> dMatrix = cluster->getdMatrix(launchPts);
    vector<vector<Pt*>> routes = greedyTenderCluster(*soln.tenderSolns[c], dMatrix);                // Run greedy 2-Opt on routes
    FullSoln new_soln = FullSoln(soln, routes, c);
    /* PRINT ROUTES AND DISTS FOR EACH SUB - TOUR!! */
    if (print) {
        printf("\nOriginal route:\t%.2f\n", soln.getTotalDist());
        //for (const auto& tender : new_soln.tenderSolns) {
        //    printTenderRoutes(tender);
        //}
    }
    return new_soln;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

// Simulated Annealing based on in/out mutator
FullSoln SA_fn(const FullSoln initialSolution,
    function<FullSoln(const FullSoln currentSolution, const int, const vector<int>, const bool)> mutator,
    const SAparams sa_params,/* SAlog& log, */bool print = false) {
    double temp = sa_params.initial_temp;           // is this redundant? - temp is updated in SAlog
    vector<int> randomness;
    for (int i = 1; i < sa_params.num_iterations; i++) { randomness.push_back(i); }
    FullSoln best = initialSolution;
    FullSoln incumbent = initialSolution;
    FullSoln proposed = initialSolution;
    double dist_best_saved = best.getTotalDist();
    double dist_best = best.getTotalDist();
    double dist_incumbent = incumbent.getTotalDist();
    double dist_proposed = proposed.getTotalDist();
    printf("\n\tBEST\t\tINCUMBENT\tPROPOSED\t\t\t\tTEMP");
    srand(42);      // set random seed
    vector<double> sa_new, sa_current, sa_best, sa_temp;
    //log = SAlog(temp);
    for (int iter_num = 1; iter_num < sa_params.num_iterations; ++iter_num) {
        if (best.msSoln.launchPts.size() == 0) { throw runtime_error("Launch points not set!"); }
        printf("\n%d\t%.3f\t%.3f\t\t\t\t\t\t%.2e", iter_num, best.getTotalDist(), dist_incumbent, temp);
        proposed = mutator(incumbent, iter_num, randomness, print);
        //printf("\t\tstop here");
        dist_incumbent = incumbent.getTotalDist();
        dist_best = best.getTotalDist();
        dist_proposed = proposed.getTotalDist();
        printf("\n%d\t%.3f\t%.3f\t%.3f", iter_num, dist_best, dist_incumbent, dist_proposed);

        if (accept_new_solution(dist_incumbent, dist_proposed, temp)) {
            incumbent = proposed;       // overwrite old solution, but have been set as const...
            dist_incumbent = incumbent.getTotalDist();
            printf("\tACCEPTED Proposed soln\n\t%.3f\t%.3f\t%.3f", dist_best, dist_incumbent, dist_proposed);
            if (dist_proposed < dist_best) {
                best = proposed; 
                dist_best = best.getTotalDist();
                printf("\n\t!!IMPROVED!! Proposed soln\n\t%.3f\t%.3f\t%.3f", dist_best, dist_incumbent, dist_proposed);
            }
        }
        temp *= sa_params.cooling_rate;
        sa_new.push_back(dist_proposed);
        sa_current.push_back(dist_incumbent);
        sa_best.push_back(dist_best);
        sa_temp.push_back(temp);       
    }
    best.setSAlog(sa_new, sa_current, sa_best, sa_temp);        //.sa_log = new SAlog(sa_new, sa_current, sa_best, sa_temp);
    // log is not used correctly! - need to update log with new values iteratively
    //log = SAlog(dist_proposed, dist_incumbent, dist_best, temp);        //update log
    //best.sa_log = log;
    printf("\n\n\tBEST\t\tINITIAL\t\t\tTEMP");
    printf("\n\t%.3f\t%.3f\t%.2e", dist_best, dist_best_saved, temp);
    FullSoln best_new = best;           // WHY is this line necessary?!
    return best_new;
}

// Shell function setting up SA, and calling SA_fn with specified mutator (IN/OUT)
FullSoln SwapShell(const FullSoln soln_prev_best, bool in_out, 
    //int num_iterations = 10000, double initial_temperature = 200, double cooling_rate = 0.999,
    bool print_stats = false, bool csv_print = false, bool SA_print = true) {   //in_out = 1; // 0 = OUT, 1 = IN
    if (in_out == 0) {      printf("\n\nWithOUT Cluster\n"); }
    else if (in_out == 1) { printf("\n\nWithIN Cluster\n"); }
    else {          throw runtime_error("Error: Invalid in_out argument provided"); }    
    printf("---------- CLUST_OPT_D_TOURS - Simulated Annealing ----------\n");
    
    //SAlog log = SAlog(/*initial_temperature*/);
    FullSoln best(soln_prev_best);
    double dist_best = best.getTotalDist();
    //SAparams                  (num_iter, init_temp, cooling_rate)
    SAparams sa_params = SAparams(10000, 0.5 * dist_best, 0.99);
        //SAparams(5000, 0.5 * dist_best, 0.98);
        //SAparams(1000, 0.5 * dist_best, 0.9);
    if (in_out == 0) {      //best = OUT_ClusterSwaps(soln_prev_best, 0, true);
        //SAparams sa_params = SAparams(10000, 0.75*dist_best, 0.99/*pow(dist_best*5E-23,1/5000)*//*0.98*/);//(4000, 4000, 0.998);// initial temp ~ 2000 -> 
        best = SA_fn(soln_prev_best, OUT_ClusterSwaps, sa_params/*, log*/); 
        printf("\n^^ OUT SWAPS ^^\n");
    }
    else {                  //best = IN_ClusterSwaps(soln_prev_best, 0, true);
        //SAparams sa_params = SAparams(10000, 0.75*dist_best, 0.99);
        best = SA_fn(soln_prev_best, IN_ClusterSwaps, sa_params/*, log*//*, true*/);
        printf("\n^^ IN SWAPS ^^\n");
    }

    printf("\n------------- ^^ CLUST_OPT_D_TOURS ^^ --------------\n");
    return best;
}