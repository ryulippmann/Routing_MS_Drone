#pragma once
#include <fstream>

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

bool accept_new_solution(double current_dist, double proposed_dist, double temperature) {
    if (proposed_dist < current_dist) return true;
    else return ((rand() / static_cast<double>(RAND_MAX)) < exp((current_dist - proposed_dist) / temperature));  // probability of accepting new solution (if worse)
}

/////////////////////////////////////////////////////////////////////////

/// <summary>
/// Function randomly returning pair of routes or clusters to swap
/// If duplicate_rule = false, then can be same
/// </summary>
/// <param name="size_a"></param>
/// <param name="size_b">= -1</param>
/// <param name="duplicate_rule">= True</param>
/// <returns>pair of random indices [0, size_a) and [0, size_b) for swapping routes or stops</returns>
pair<int, int> randSwapChoice(size_t size_a, /*int randomSeed = 12345, */size_t size_b = -1, bool duplicate_rule = true) {       // RANDOMLY CHOOSE WHICH d_route TO SWAP
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
int randChoice(const size_t size/*, int randomSeed = 12345*/, bool clust_choice = false) {
    if (clust_choice) return getRandomNumber(size);
    else return getRandomNumber(size - 3/*, randomSeed*/) + 1;
}

vector<Pt*> UpdateLaunchPts(const vector<ClusterSoln*> clusters, bool print = false) {
    if (print) printf("\n---- SET LAUNCH POINTS ----\n\tID\t(  x  ,  y  )\n");
    vector<Pt*> launchPts = SetWeightedLaunchPts(clusters);
    // add depot as first launch point
    if (print) {
        printf("\tID\t(  x  ,  y  )\n");
        cout << string(30, '-') << "\n";
        printf("\t%d\t( %2.2f, %2.2f)\n", INST.ms.depot.ID, INST.ms.depot.x, INST.ms.depot.y);
        for (const auto& stop : launchPts) {
            printf("\t%d\t( %.2f, %.2f)\n", stop->ID, stop->x, stop->y);
        } printf("\n");
        cout << string(30, '-') << "\n";
        double total_dist = calculatePtDistance(INST.ms.depot, launchPts[0]);
        printf("\t%.2f ", total_dist);
        for (int i = 0; i < launchPts.size() - 1; i++) {
            double leg_dist = calculatePtDistance(launchPts[i], launchPts[i + 1]);
            printf("+\t%.2f ", leg_dist);
            total_dist += leg_dist;
        }
        printf("+\t%.2f ", calculatePtDistance(INST.ms.depot, launchPts.back()));
        total_dist += calculatePtDistance(INST.ms.depot, launchPts.back());
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
/*pair<DroneSoln, DroneSoln> */void random_d_out_Swap(DroneSoln& drone_a, DroneSoln& drone_b,/*pair<DroneSoln, DroneSoln> drones, */
    bool swap_print = false) {//, int iteration) { // d_route = ; tours = d_tours in this cluster
    /* RANDOMLY CHOOSE WHICH d_route TO SWAP */
    pair<int, int> s = randSwapChoice(  drone_a.routes.size(), //iteration, 
                                        drone_b.routes.size(), false);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    // RANDOMLY CHOOSE WHICH d_route TO SWAP FROM
    int from_stop = randChoice(drone_a.routes[s.first].size()/*, iteration*/);        // choose stop number to swap out from current d_routes
    Pt* from_node = drone_a.routes[s.first][from_stop];
    if (from_stop <= 0 || from_stop > drone_a.routes[s.first].size() - 3) { throw runtime_error("Error: Invalid from_stop chosen"); }
    // ^^ only true if drone route contains launchpts - this is to safeguard them being swapped...
    
    // RANDOMLY CHOOSE WHICH d_route TO SWAP TO
    int to_stop = randChoice(drone_b.routes[s.second].size()/*, iteration*/);         // choose stop number to swap out from current d_routes
    Pt* to_node = drone_b.routes[s.second][to_stop];
    if (to_stop <= 0 || to_stop > drone_b.routes[s.second].size() - 3) { throw runtime_error("Error: Invalid to_stop chosen"); }
    // ^^ only true if drone route contains launchpts - this is to safeguard them being swapped...

    // SWAP:REEFS in clusters and NODES in routes
    swap(drone_a.cluster.reefs[findIndexByID(drone_a.routes[s.first][from_stop]->ID, drone_a.cluster.reefs) -1],
        drone_b.cluster.reefs[findIndexByID(drone_b.routes[s.second][to_stop]->ID, drone_b.cluster.reefs)-1]);
    swap(drone_a.routes[s.first][from_stop], 
        drone_b.routes[s.second][to_stop]);

    if (swap_print) {
        printf("\nSWAP:\td_routes\tindex\tnode\tReef_id\t\t(x,y)");
        printf("\nc\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.first,   from_stop,  from_node->ID,  from_node->x,   from_node->y);
        printf("\nd\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.second,  to_stop,    to_node->ID,    to_node->x,     to_node->y);
    }
    return;
}

/// <summary>
/// /// </summary>
/// <param name="soln">: FullSoln incumbent</param>
/// <param name="iteration"></param>
/// <param name="print">= False</param>
/// <returns></returns>
FullSoln OUT_ClusterSwaps(const Problem& inst, const FullSoln& soln/*, int iteration*//*vector<int> randoms, */, bool print = false) {
    if (print) printf("---- OUT_Swap ----");
    pair<int, int> c = randSwapChoice(soln.msSoln.clusters.size()/*, iteration*/);    // generate swap pair of clusters
    if (print) printf("\nSwap clusters:\t\t%d\tand\t%d", c.first, c.second);

    vector<DroneSoln*> droneSolns = soln.droneSolns;
    vector<ClusterSoln*> clusters = soln.msSoln.clusters;

    random_d_out_Swap(*droneSolns[c.first], *droneSolns[c.second]);//,iteration, false);

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
        routes = make_pair( greedyDroneCluster(*droneSolns[c.first], dMatrix.first),
                            greedyDroneCluster(*droneSolns[c.second], dMatrix.second));
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

    int from_stop = randChoice(drone.routes[s.first].size()/*, iteration*/);     // randomly choose stop number to swap out from current/chosen d_routes
    if (swap_print) printf("\nSwap From\tindex %d: node = %d\t", from_stop, drone.routes[s.first][from_stop]->ID);
    if (from_stop <= 0 || from_stop > drone.routes[s.first].size()-3) {throw runtime_error ("Error: Invalid from_stop chosen");}

    int to_stop = randChoice(drone.routes[s.second].size()/*, iteration*/);      // randomly choose stop number to swap out from current d_routes
    if (swap_print) printf("\nTo\t\tindex %d: node = %d\t", to_stop, drone.routes[s.second][to_stop]->ID);
    if (to_stop <= 0 || to_stop > drone.routes[s.second].size() - 3) { throw runtime_error("Error: Invalid to_stop chosen"); }

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
/// MODIFY Fullsoln soln (not ref) and return the swapped copy to save as proposed - mutated cluster_index c to swap within
/// </summary>
/// <param name="soln"></param>
/// <param name="iteration"></param>
/// <param name="print"></param>
/// <returns></returns>
FullSoln IN_ClusterSwaps(const Problem& inst, const FullSoln& soln, /*int iteration, *//*vector<int> randoms, */bool print = false) {
    vector<vector<Pt*>> routes;
    //int c;
    //int num_it = 1;                             // set number of swaps per iteration
    //for (int i = 0; i < num_it; i++) {
    int c = randChoice(soln.msSoln.clusters.size()/*, iteration*/, true);    // generate random CLUSTER to swap within
    if (print) {
        cout << "\n" << string(50, '~') << "\n";
        printf("\nSwap cluster:\t\t%d\n", c);
        printDroneRoutes(soln.droneSolns[c]);
    }
    // RANDOMLY SWAP WITHIN CLUSTER
    random_d_in_Swap(*soln.droneSolns[c]/*, iteration*/);          // DONT CREATE COPY, as soln is NOT reference
    if (print) { printDroneRoutes(soln.droneSolns[c]); }

    // these are only created to feed into greedyDroneCluster
    ClusterSoln* cluster = soln.msSoln.clusters[c];
    pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln.launchPts[c], soln.msSoln.launchPts[c + 1]);
    vector<vector<double>> dMatrix = cluster->getdMatrix(launchPts);
    routes = greedyDroneCluster(*soln.droneSolns[c], dMatrix);                // Run greedy 2-Opt on routes
    //}
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
FullSoln SwapRandomly(Problem inst, const FullSoln soln_prev_best, SAparams sa_params, int run_iteration,
    //int num_iterations = 10000, double initial_temperature = 200, double cooling_rate = 0.999,
    bool print_stats = false, bool csv_print = false, bool SA_print = true) {   //in_out = 1; // 0 = OUT, 1 = IN
    printf("\n\n---------- RANDOM IN/OUT Cluster Swaps - Simulated Annealing ----------\n");
    FullSoln best(soln_prev_best);
    double dist_best = best.getTotalDist();
    FullSoln incumbent = soln_prev_best;
    double dist_initial = dist_best;

    double temp = sa_params.initial_temp;           // is this redundant? - temp is updated in SAlog
    
    srand(42);      // set random seed
    vector<double> sa_new, sa_current, sa_best, sa_temp;
    //int in_swaps = 0, out_swaps = 0;
    printf("\n\tBEST\t\t\tTEMP\t\t\tPROPOSED\t\tINCUMBENT");

    for (int iter_num = 0; iter_num < sa_params.num_iterations+1; ++iter_num) {
        if (best.msSoln.launchPts.size() == 0) { throw runtime_error("Launch points not set!"); break; }
        FullSoln proposed = incumbent;
        bool in_out = rand() % 2;       // randomly choose IN or OUT cluster swap
        if (in_out) {
            proposed = IN_ClusterSwaps(inst, incumbent, /*iter_num, *//*randomness, */print_stats);
            //in_swaps += 1;
		}
		else {
            proposed = OUT_ClusterSwaps(inst, incumbent, /*iter_num, *//*randomness, */print_stats);
            //out_swaps += 1;
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
                printf("\n\t%.3f\t!!IMPROVED!! Proposed soln", dist_best);
                if (csv_print) {
                    csvUpdate(best, in_out, iter_num/*in_swaps+out_swaps*/, run_iteration);
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
    if (dist_best == dist_initial) printf("\n\n\tNO IMPROVEMENT MADE\n");
    else printf("\n\n\tBEST\t\t\tINITIAL\t\t\tTEMP\n\t%.3f\t\t%.3f\t\t%.2e", dist_best, dist_initial, temp);
    FullSoln best_new = best;           // IS this line necessary?!

    printf("\n------------- ^^ CLUST_OPT_D_TOURS ^^ --------------\n");
    return best_new;
}
