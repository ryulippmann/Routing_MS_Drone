#pragma once
#include <fstream>

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

bool accept_new_solution(double current_dist, double proposed_dist, double temperature) {
    /*if (proposed_dist < current_dist) return true;
    else */return ((rand() / static_cast<double>(RAND_MAX)) < exp((current_dist - proposed_dist) / temperature));  // probability of accepting new solution (if worse)
}

/////////////////////////////////////////////////////////////////////////

/// <summary>
/// Function randomly returning pair of routes or clusters to swap
/// </summary>
/// <param name="size_a"></param>
/// <param name="size_b">= -1</param>
/// <param name="duplicate_rule">= True</param>
/// <returns>pair of random indices [0, size_a) and [0, size_b) for swapping routes or stops</returns>
pair<int, int> randSwapChoice(size_t size_a, /*int randomSeed = 12345, */size_t size_b = -1, bool duplicate_rule = true) {       // RANDOMLY CHOOSE WHICH d_route TO SWAP
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

/// <summary>
/// return random number [1, size-2] for: 
/// - swapping stops within route
/// - choosing which cluster to swap within
/// </summary>
/// <param name="size"></param>
/// <param name="clust_choice"></param>
/// <returns></returns>
int randChoice(size_t size/*, int randomSeed = 12345*/, bool clust_choice = false) {
    if (clust_choice) return getRandomNumber(size);
    else return getRandomNumber(size - 3/*, randomSeed*/) + 1;
}

vector<Pt*> UpdateLaunchPts(const vector<ClusterSoln*> clusters, bool print = false) {
    if (print) printf("\n---- SET LAUNCH POINTS ----\n\tID\t(  x  ,  y  )\n");
    vector<Pt*> launchPts;
    // add depot as first launch point
    launchPts.push_back(new Pt(
        (inst.ms.depot.x + clusters[0]->getCentroid().x) / 2,
        (inst.ms.depot.y + clusters[0]->getCentroid().y) / 2));
    for (int c = 0; c < clusters.size() - 1; c++) {
        launchPts.push_back(new Pt(        // sum adjacent clusters x,y's to calc launchpts and 
            (clusters[c]->getCentroid().x + clusters[c + 1]->getCentroid().x) / 2,
            (clusters[c]->getCentroid().y + clusters[c + 1]->getCentroid().y) / 2));
    }
    launchPts.push_back(new Pt(
        (inst.ms.depot.x + clusters.back()->getCentroid().x) / 2,
        (inst.ms.depot.y + clusters.back()->getCentroid().y) / 2));
    if (print) {
        printf("\tID\t(  x  ,  y  )\n");
        cout << string(30, '-') << "\n";
        printf("\t%d\t( %2.2f, %2.2f)\n", inst.ms.depot.ID, inst.ms.depot.x, inst.ms.depot.y);
        for (const auto& stop : launchPts) {
            printf("\t%d\t( %.2f, %.2f)\n", stop->ID, stop->x, stop->y);
        } printf("\n");
        cout << string(30, '-') << "\n";
        double total_dist = calculatePtDistance(inst.ms.depot, launchPts[0]);
        printf("\t%.2f ", total_dist);
        for (int i = 0; i < launchPts.size() - 1; i++) {
            double leg_dist = calculatePtDistance(launchPts[i], launchPts[i + 1]);
            printf("+\t%.2f ", leg_dist);
            total_dist += leg_dist;
        }
        printf("+\t%.2f ", calculatePtDistance(inst.ms.depot, launchPts.back()));
        total_dist += calculatePtDistance(inst.ms.depot, launchPts.back());
        printf("\n\t\t= %.2f", total_dist);
    }
    return launchPts;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/// <summary>
/// very similar to random_d_in_Swap... possibly duplicate or overload function
/// </summary>
/// <param name="drones"></param>
/// <param name="iteration"></param>
/// <param name="swap_print"></param>
/// <returns></returns>
pair<DroneSoln, DroneSoln> random_d_out_Swap(pair<DroneSoln, DroneSoln> drones, bool swap_print = false) {//, int iteration) { // d_route = ; tours = d_tours in this cluster
    /* RANDOMLY CHOOSE WHICH d_route TO SWAP */
    pair<int, int> s = randSwapChoice(  drones.first.routes.size(), //iteration, 
                                        drones.second.routes.size(), false);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    // pair<int, int> stops = randSwapChoice(..., ..., false); // make sure indices are correct. i.e. +1 ?

    /* RANDOMLY CHOOSE WHICH d_route TO SWAP FROM*/
    int from_stop = randChoice(drones.first.routes[s.first].size()/*, iteration*/);        // choose stop number to swap out from current d_routes
    Pt* from_node = drones.first.routes[s.first][from_stop];
    if (from_stop == 0 || from_stop > drones.first.routes[s.first].size() - 3) { throw runtime_error("Error: Invalid from_stop chosen"); }
    // ^^ this only true if drone route contains launchpts - this is to safeguard them being swapped...
    
    /* RANDOMLY CHOOSE WHICH d_route TO SWAP TO*/
    int to_stop = randChoice(drones.second.routes[s.second].size()/*, iteration*/);         // choose stop number to swap out from current d_routes
    Pt* to_node = drones.second.routes[s.second][to_stop];
    if (to_stop == 0 || to_stop > drones.second.routes[s.second].size() - 3) { throw runtime_error("Error: Invalid to_stop chosen"); }
    // ^^ this only true if drone route contains launchpts - this is to safeguard them being swapped...

    swap(drones.first.cluster.reefs[findIndexByID(drones.first.routes[s.first][from_stop]->ID, drones.first.cluster.reefs)-1],
        drones.second.cluster.reefs[findIndexByID(drones.second.routes[s.second][to_stop]->ID, drones.second.cluster.reefs)-1]);
    swap(drones.first.routes[s.first][from_stop], drones.second.routes[s.second][to_stop]);

    if (swap_print) {
        printf("\nSWAP:\td_routes\tindex\tnode\tReef_id\t\t(x,y)");
        printf("\nc\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.first,   from_stop,  from_node->ID,  from_node->x,   from_node->y);
        printf("\nd\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.second,  to_stop,    to_node->ID,    to_node->x,     to_node->y);
    }
    return drones;
}

/// <summary>
/// /// </summary>
/// <param name="soln">: FullSoln incumbent</param>
/// <param name="iteration"></param>
/// <param name="print">= False</param>
/// <returns></returns>
FullSoln OUT_ClusterSwaps(FullSoln soln/*, int iteration*//*vector<int> randoms, */, bool print = false) {
    if (print) printf("---- OUT_Swap ----");
    pair<int, int> c = randSwapChoice(soln.msSoln.clusters.size()/*, iteration*/);    // generate swap pair of clusters
    if (print) printf("\nSwap clusters:\t\t%d\tand\t%d", c.first, c.second);

    pair<DroneSoln, DroneSoln>
        drones = random_d_out_Swap(make_pair(*soln.droneSolns[c.first], *soln.droneSolns[c.second]));//,iteration, false);

    vector<DroneSoln*> droneSolns;
    for (int i = 0; i < soln.droneSolns.size(); i++) {
        if (i == c.first)       { droneSolns.push_back(&drones.first); }
        else if (i == c.second) { droneSolns.push_back(&drones.second); }
        else                    { droneSolns.push_back(soln.droneSolns[i]); }
    }
    vector<ClusterSoln*> clusters;
    for (int i = 0; i < droneSolns.size(); i++) { clusters.push_back(&(droneSolns[i]->cluster)); }

    // update droneSolns with new launchPts & in routes
    vector<Pt*>
        launchPts = UpdateLaunchPts(clusters/*, true*/);
    for (int d = 0; d < droneSolns.size(); d++) {
        droneSolns[d]->launchPts = make_pair(launchPts[d], launchPts[d + 1]);
        for (auto& route : droneSolns[d]->routes) {
            route[0] = launchPts[d];
            route[route.size() - 2] = launchPts[d + 1];
            route[route.size() - 1] = launchPts[d];
        }
    }

    //// UPDATE droneSolns with new routes ////
    pair <vector<vector<double>>, vector<vector<double>>>    // dMatrix feeds into routes
        dMatrix = make_pair(droneSolns[c.first]->cluster.getdMatrix(make_pair(launchPts[c.first], launchPts[c.first + 1])),
                            droneSolns[c.second]->cluster.getdMatrix(make_pair(launchPts[c.second], launchPts[c.second + 1])));
    pair <vector<vector<Pt*>>, vector<vector<Pt*>>>         // update routes with Gd before assessing solution
        routes = make_pair( greedyDroneCluster(drones.first, dMatrix.first),
                            greedyDroneCluster(drones.second, dMatrix.second));
    droneSolns[c.first]->routes = routes.first;            // update clusters with new routes
    droneSolns[c.second]->routes = routes.second;

    MSSoln msSoln = MSSoln(clusters, launchPts);
    FullSoln new_soln = FullSoln(msSoln, droneSolns);      // create new FullSoln with updated clusters and launchPts
    if (print) printf("-- ^^ OUT_Swap ^^ --");
    return new_soln;
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/// <summary>
/// SWAP DroneSoln routes directly by ref
/// </summary>
/// <param name="drone"></param>
/// <param name="swap_print"></param>
void random_d_in_Swap(DroneSoln& drone, /*int iteration, */bool swap_print = false) { // d_route = ; tours = d_tours in this cluster
    pair<int, int> s = randSwapChoice(drone.routes.size()/*, iteration*/);   // RANDOMLY CHOOSE WHICH d_routes TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    // pair<int, int> stops = randSwapChoice(..., ..., false); // make sure indices are correct. i.e. +1 ?

    int from_stop = randChoice(drone.routes[s.first].size()/*, iteration*/);     // randomly choose stop number to swap out from current/chosen d_routes
    if (swap_print) printf("\nSwap From\tindex %d: node = %d\t", from_stop, drone.routes[s.first][from_stop]->ID);
    if (from_stop == 0 || from_stop > drone.routes[s.first].size()-3) {throw runtime_error ("Error: Invalid from_stop chosen");}

    int to_stop = randChoice(drone.routes[s.second].size()/*, iteration*/);      // randomly choose stop number to swap out from current d_routes
    if (swap_print) printf("\nTo\t\tindex %d: node = %d\t", to_stop, drone.routes[s.second][to_stop]->ID);
    if (to_stop == 0 || to_stop > drone.routes[s.second].size() - 3) { throw runtime_error("Error: Invalid to_stop chosen"); }

    swap(drone.routes[s.first][from_stop], drone.routes[s.second][to_stop]);

    if (swap_print) {
        for (const auto& d_route : drone.routes) {
            printf("\n\t");
            for (const auto& node : d_route) { cout << node << "\t"; }
        }
    } //cout << "\n";
    return;
}

/// <summary>
/// print drone routes and dists - based on droneSoln* drone
/// Only called in IN_ClusterSwaps
/// </summary>
/// <param name="drone"></param>
void printDroneRoutes(const DroneSoln* drone) {
	for (const auto& route : drone->routes) {
        for (auto& node : route) {
            printf("\t%d ->", node->ID);
        }
        printf("\nDrone route dist:\t%f\n", drone->getDroneRouteDist(route));
    }
}

/// <summary>
/// MODIFY Fullsoln soln (not ref) and return the swapped copy to save as proposed - mutated cluster_index c to swap within
/// </summary>
/// <param name="soln"></param>
/// <param name="iteration"></param>
/// <param name="print"></param>
/// <returns></returns>
FullSoln IN_ClusterSwaps(FullSoln soln, /*int iteration, *//*vector<int> randoms, */bool print = false) {
    vector<vector<Pt*>> routes;
    int c;

    //double scaling_param = 0.04;                                        // scaling parameter
    //double num_it = (1/(log(iteration) + 1)) * no_pts * scaling_param;  // number of swaps to make
    //num_it = num_it > 1 ? num_it : 1;       	                        // ensure at least 1 swap is made    
	int num_it = 1;                             // set number of swaps = 1
    for (int i = 0; i < num_it; i++) {
        c = randChoice(soln.msSoln.clusters.size()/*, iteration*/, true);    // generate random CLUSTER to swap within
        //throw runtime_error("Not all clusters swappable! randChoice limits choice [1,clusters.size()-3]");
        if (print) {
            cout << "\n" << string(50, '~') << "\n";
            printf("\nSwap cluster:\t\t%d\n", c);
            printDroneRoutes(soln.droneSolns[c]);
        }
        random_d_in_Swap(*soln.droneSolns[c]/*, iteration*/);          // DONT CREATE COPY, as soln is NOT reference
        if (print) { printDroneRoutes(soln.droneSolns[c]); }

        // these are only created to feed into greedyDroneCluster
        ClusterSoln* cluster = soln.msSoln.clusters[c];
        pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln.launchPts[c], soln.msSoln.launchPts[c + 1]);
        vector<vector<double>> dMatrix = cluster->getdMatrix(launchPts);
        routes = greedyDroneCluster(*soln.droneSolns[c], dMatrix);                // Run greedy 2-Opt on routes
        /* PRINT ROUTES AND DISTS FOR EACH SUB - TOUR!! */
    }
    FullSoln new_soln = FullSoln(soln, routes, c);
        if (print) { printf("\nOriginal route:\t%.2f\n", soln.getTotalDist());
            for (const auto& drone : new_soln.droneSolns) { printDroneRoutes(drone); }
        }
	return new_soln;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/// <summary>
/// Shell function setting up SA, and calling SA_fn with specified mutator 
/// Randomly swapping by In/Out cluster with 50/50 probability
/// </summary>
/// <param name="soln_prev_best"></param>
/// <param name="in_out"></param>
/// <param name="print_stats"></param>
/// <param name="csv_print"></param>
/// <param name="SA_print"></param>
/// <returns></returns>
FullSoln SwapRandomly(const FullSoln soln_prev_best, SAparams sa_params,
    //int num_iterations = 10000, double initial_temperature = 200, double cooling_rate = 0.999,
    bool print_stats = false, bool csv_print = false, bool SA_print = true) {   //in_out = 1; // 0 = OUT, 1 = IN
    printf("\n\n---------- RANDOM IN/OUT Cluster Swaps - Simulated Annealing ----------\n");
    //SAlog log = SAlog(/*initial_temperature*/);
    FullSoln best(soln_prev_best);
    double dist_best = best.getTotalDist();
    FullSoln incumbent = soln_prev_best;
    double dist_initial = dist_best;

    double temp = sa_params.initial_temp;           // is this redundant? - temp is updated in SAlog
    
    srand(42);      // set random seed
    vector<double> sa_new, sa_current, sa_best, sa_temp;
    //log = SAlog(temp);
    int in_swaps = 0, out_swaps = 0;
    printf("\n\tBEST\t\t\tTEMP\t\t\tPROPOSED\t\tINCUMBENT");
    // introduce while loop to ensure solution has found steady-state?
    //while (best_dist.size() < 3 || best_dist.back() != best_dist.at(best_dist.size() - 3))
    for (int iter_num = 0; iter_num < sa_params.num_iterations+1; ++iter_num) {
        if (best.msSoln.launchPts.size() == 0) { throw runtime_error("Launch points not set!"); break; }
        FullSoln proposed = incumbent;
        bool in_out = rand() % 2;       // randomly choose IN or OUT cluster swap
        if (in_out) {
            proposed = IN_ClusterSwaps(incumbent, /*iter_num, *//*randomness, */print_stats);
            in_swaps += 1;
		}
		else {
            proposed = OUT_ClusterSwaps(incumbent, /*iter_num, *//*randomness, */print_stats);
            out_swaps += 1;
		}

        double dist_proposed = proposed.getTotalDist(), dist_incumbent = incumbent.getTotalDist();
        printf("\n%d\t%5.3f\t\t%.2e\t\t%5.3f\t\t%5.3f\t%s", iter_num, dist_best, temp, dist_proposed, dist_incumbent, in_out == 1 ? "IN" : "OUT");
        if (accept_new_solution(dist_incumbent, dist_proposed, temp)) {
            incumbent = proposed;       // overwrite old solution, but have been set as const...
            dist_incumbent = incumbent.getTotalDist();
            printf("\tACCEPTED Proposed soln");
            if (dist_proposed < dist_best) {
                best = proposed;
                dist_best = best.getTotalDist();
                printf("\n\t!!IMPROVED!! Proposed soln\t\t%.3f", dist_best);
                if (csv_print) {
                    csvUpdate(best, in_out, in_swaps+out_swaps);
                    //if (in_out) csvUpdate_IN(best);
                    //else csvUpdate_OUT(best);
                }
            }
        }
        
        temp *= sa_params.cooling_rate;
        sa_new.push_back(dist_proposed);
        sa_current.push_back(dist_incumbent);
        sa_best.push_back(dist_best);
        sa_temp.push_back(temp);
    }
    best.setSAlog(sa_new, sa_current, sa_best, sa_temp, sa_params);        //.sa_log = new SAlog(sa_new, sa_current, sa_best, sa_temp);
    // log is not used correctly! - need to update log with new values iteratively
    //log = SAlog(dist_proposed, dist_incumbent, dist_best, temp);        //update log
    //best.sa_log = log;
    if (dist_best == dist_initial) printf("\n\n\tNO IMPROVEMENT MADE\n");
    else printf("\n\n\tBEST\t\t\tINITIAL\t\t\tTEMP\n\t%.3f\t\t%.3f\t\t%.2e", dist_best, dist_initial, temp);
    FullSoln best_new = best;           // IS this line necessary?!

    //if (in_out == 0) {      //best = OUT_ClusterSwaps(soln_prev_best, 0, true);
    //    best = SA_fn(soln_prev_best, OUT_ClusterSwaps, sa_params/*, log*/);
    //    printf("\n^^ OUT SWAPS ^^\n");
    //}
    //else {                  //best = IN_ClusterSwaps(soln_prev_best, 0, true);
    //    best = SA_fn(soln_prev_best, IN_ClusterSwaps, sa_params/*, log*//*, true*/);
    //    printf("\n^^ IN SWAPS ^^\n");
    //}

    printf("\n------------- ^^ CLUST_OPT_D_TOURS ^^ --------------\n");
    return best_new;
}

///// <summary>
///// Simulated Annealing based on in/out mutator
///// </summary>
///// <param name="initialSolution"></param>
///// <param name="mutator">: FUNCTION taken as IN_ClusterSwaps or OUT_ClusterSwaps</param>
///// <param name="sa_params"></param>
///// <param name="print"></param>
///// <returns>FullSoln best_new</returns>
//FullSoln SA_fn(const FullSoln initialSolution,
//    function<FullSoln(const FullSoln currentSolution, const int, /*const vector<int>, */const bool)> mutator,
//    const SAparams sa_params,/* SAlog& log, */bool print = false) {
//    double temp = sa_params.initial_temp;           // is this redundant? - temp is updated in SAlog
//    //vector<int> randomness;
//    //for (int i = 1; i < sa_params.num_iterations; i++) { randomness.push_back(i); }
//    FullSoln best = initialSolution;
//    FullSoln incumbent = initialSolution;
//    double dist_best = best.getTotalDist();
//    double dist_initial = dist_best;
//    double dist_incumbent = dist_best;
//    double dist_proposed = dist_best;
//    printf("\n\tBEST\t\t\tINCUMBENT\t\tTEMP\t\t\tPROPOSED");
//    srand(42);      // set random seed
//    vector<double> sa_new, sa_current, sa_best, sa_temp;
//    //log = SAlog(temp);
//    for (int iter_num = 1; iter_num < sa_params.num_iterations + 1; ++iter_num) {
//        if (best.msSoln.launchPts.size() == 0) { throw runtime_error("Launch points not set!"); }
//        printf("\n%d\t%5.3f\t\t%5.3f\t\t%.2e", iter_num, dist_best, dist_incumbent, temp);
//        FullSoln proposed = mutator(incumbent, iter_num, /*randomness, */print);
//        dist_incumbent = incumbent.getTotalDist();
//        dist_proposed = proposed.getTotalDist();
//        printf("\t\t%5.3f", dist_proposed);
//
//        if (accept_new_solution(dist_incumbent, dist_proposed, temp)) {
//            incumbent = proposed;       // overwrite old solution, but have been set as const...
//            dist_incumbent = incumbent.getTotalDist();
//            printf("\tACCEPTED Proposed soln");
//            if (dist_proposed < dist_best) {
//                best = proposed; 
//                dist_best = best.getTotalDist();
//                printf("\n\t!!IMPROVED!! Proposed soln\t%.3f\t\t%.3f\t\t%.3f", dist_best, dist_incumbent, dist_proposed);
//            }
//        }
//        temp *= sa_params.cooling_rate;
//        sa_new.push_back(dist_proposed);
//        sa_current.push_back(dist_incumbent);
//        sa_best.push_back(dist_best);
//        sa_temp.push_back(temp);       
//    }
//    best.setSAlog(sa_new, sa_current, sa_best, sa_temp);        //.sa_log = new SAlog(sa_new, sa_current, sa_best, sa_temp);
//    // log is not used correctly! - need to update log with new values iteratively
//    //log = SAlog(dist_proposed, dist_incumbent, dist_best, temp);        //update log
//    //best.sa_log = log;
//    if (dist_best == dist_initial) printf("\n\n\tNO IMPROVEMENT MADE\n");
//    else printf("\n\n\tBEST\t\t\tINITIAL\t\t\tTEMP\n\t%.3f\t\t%.3f\t\t%.2e", dist_best, dist_initial, temp);
//    FullSoln best_new = best;           // WHY is this line necessary?!
//    return best_new;
//}
//
///// <summary>
///// Shell function setting up SA, and calling SA_fn with specified mutator (IN/OUT)
///// </summary>
///// <param name="soln_prev_best"></param>
///// <param name="in_out"></param>
///// <param name="print_stats">= false</param>
///// <param name="csv_print">= false</param>
///// <param name="SA_print">= true</param>
///// <returns></returns>
//FullSoln SwapShell(const FullSoln soln_prev_best, bool in_out, 
//    //int num_iterations = 10000, double initial_temperature = 200, double cooling_rate = 0.999,
//    bool print_stats = false, bool csv_print = false, bool SA_print = true) {   //in_out = 1; // 0 = OUT, 1 = IN
//    if (in_out == 0) {      printf("\n\nWithOUT Cluster\n"); }
//    else if (in_out == 1) { printf("\n\nWithIN Cluster\n"); }
//    else {          throw runtime_error("Error: Invalid in_out argument provided"); }    
//    printf("---------- CLUST_OPT_D_TOURS - Simulated Annealing ----------\n");
//    
//    //SAlog log = SAlog(/*initial_temperature*/);
//    FullSoln best(soln_prev_best);
//    double dist_best = best.getTotalDist();
//    //SAparams                  (num_iter, init_temp, cooling_rate)
//    int num_iter = 10000;               // fixed at 10000
//    double init_temp = 0.2 * dist_best; 
//    //double temp_diff = pow(10, -4);
//    //double final_temp = init_temp * temp_diff;//pow(10, -5);
//    //double cooling_rate = pow((temp_diff), 1 / num_iter);  //0.995;
//    double cooling_rate = 0.9995;
//    
//    SAparams sa_params = //SAparams(10000, 0.2 * dist_best, 0.995);
//        SAparams(num_iter, init_temp, cooling_rate);
//        //SAparams(5000, 0.5 * dist_best, 0.98);
//        //SAparams(1000, 0.5 * dist_best, 0.9);
//    if (in_out == 0) {      //best = OUT_ClusterSwaps(soln_prev_best, 0, true);
//        best = SA_fn(soln_prev_best, OUT_ClusterSwaps, sa_params/*, log*/); 
//        printf("\n^^ OUT SWAPS ^^\n");
//    }
//    else {                  //best = IN_ClusterSwaps(soln_prev_best, 0, true);
//        best = SA_fn(soln_prev_best, IN_ClusterSwaps, sa_params/*, log*//*, true*/);
//        printf("\n^^ IN SWAPS ^^\n");
//    }
//
//    printf("\n------------- ^^ CLUST_OPT_D_TOURS ^^ --------------\n");
//    return best;
//}
//
//FullSoln SA_fn_RANDOM(const FullSoln initialSolution,
//    function<FullSoln(const FullSoln currentSolution, const int, /*const vector<int>, */const bool)> mutator,
//    const SAparams sa_params,/* SAlog& log, */bool print = false) {
//    double temp = sa_params.initial_temp;           // is this redundant? - temp is updated in SAlog
//    //vector<int> randomness;
//    //for (int i = 1; i < sa_params.num_iterations; i++) { randomness.push_back(i); }
//    FullSoln best = initialSolution;
//    FullSoln incumbent = initialSolution;
//    double dist_best = best.getTotalDist();
//    double dist_initial = dist_best;
//    double dist_incumbent = dist_best;
//    double dist_proposed = dist_best;
//    printf("\n\tBEST\t\t\tINCUMBENT\t\tTEMP\t\t\tPROPOSED");
//    srand(42);      // set random seed
//    vector<double> sa_new, sa_current, sa_best, sa_temp;
//    //log = SAlog(temp);
//    for (int iter_num = 1; iter_num < sa_params.num_iterations + 1; ++iter_num) {
//        if (best.msSoln.launchPts.size() == 0) { throw runtime_error("Launch points not set!"); }
//        printf("\n%d\t%5.3f\t\t%5.3f\t\t%.2e", iter_num, dist_best, dist_incumbent, temp);
//        FullSoln proposed = mutator(incumbent, iter_num, /*randomness, */print);
//        dist_incumbent = incumbent.getTotalDist();
//        dist_proposed = proposed.getTotalDist();
//        printf("\t\t%5.3f", dist_proposed);
//
//        if (accept_new_solution(dist_incumbent, dist_proposed, temp)) {
//            incumbent = proposed;       // overwrite old solution, but have been set as const...
//            dist_incumbent = incumbent.getTotalDist();
//            printf("\tACCEPTED Proposed soln");
//            if (dist_proposed < dist_best) {
//                best = proposed;
//                dist_best = best.getTotalDist();
//                printf("\n\t!!IMPROVED!! Proposed soln\t%.3f\t\t%.3f\t\t%.3f", dist_best, dist_incumbent, dist_proposed);
//            }
//        }
//        temp *= sa_params.cooling_rate;
//        sa_new.push_back(dist_proposed);
//        sa_current.push_back(dist_incumbent);
//        sa_best.push_back(dist_best);
//        sa_temp.push_back(temp);
//    }
//    best.setSAlog(sa_new, sa_current, sa_best, sa_temp);        //.sa_log = new SAlog(sa_new, sa_current, sa_best, sa_temp);
//    // log is not used correctly! - need to update log with new values iteratively
//    //log = SAlog(dist_proposed, dist_incumbent, dist_best, temp);        //update log
//    //best.sa_log = log;
//    if (dist_best == dist_initial) printf("\n\n\tNO IMPROVEMENT MADE\n");
//    else printf("\n\n\tBEST\t\t\tINITIAL\t\t\tTEMP\n\t%.3f\t\t%.3f\t\t%.2e", dist_best, dist_initial, temp);
//    FullSoln best_new = best;           // WHY is this line necessary?!
//    return best_new;
//}
