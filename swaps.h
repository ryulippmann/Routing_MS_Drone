//swaps.h NEW!!

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
/// </summary>
/// <param name="size_a"></param>
/// <param name="size_b">= -1</param>
/// <param name="duplicate_rule">= True</param>
/// <returns>pair of random indices [0, size_a - 1] and [0, size_b - 1] for swapping routes or stops</returns>
pair<int, int> randSwapChoice(size_t size_a, /*int randomSeed = 12345, */size_t size_b = -1, bool duplicate_rule = true) {       // RANDOMLY CHOOSE WHICH d_route TO SWAP
    // if size = 2, and routes must be different, set one to 0, one to 1
    if (size_b == -1) { size_b = size_a; }
    int d_route_a = getRandomNumber(size_a/*, randomSeed*/);
    int d_route_b = getRandomNumber(size_b/*, randomSeed%10*/);
    //int count = 0;
    while (duplicate_rule && d_route_a == d_route_b) {        // if same routes chosen
        d_route_b = getRandomNumber(size_b/*, randomSeed + count*/);    // re-choose route to swap out TO   current d_routes
        //count++;
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

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/// <summary>
/// very similar to random_d_in_Swap... possibly duplicate or overload function
/// </summary>
/// <param name="drones"></param>
/// <param name="iteration"></param>
/// <param name="swap_print"></param>
/// <returns></returns>
pair<int, int> random_d_out_Swap(pair<DroneSoln*, DroneSoln*> drones, bool swap_print = false) {//, int iteration) { // d_route = ; tours = d_tours in this cluster
    pair<int, int> s = randSwapChoice(drones.first->routes.size(), //iteration, 
        drones.second->routes.size(), false);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    // RANDOMLY CHOOSE WHICH d_route TO SWAP FROM
    int from_stop = randChoice(drones.first->routes[s.first].size()/*, iteration*/);        // choose stop number to swap out from current d_routes
    Pt* from_node = drones.first->routes[s.first][from_stop];
    if (from_stop == 0 || from_stop > drones.first->routes[s.first].size() - 3) { throw runtime_error("Error: Invalid from_stop chosen"); }
    // ^^ only true if drone route contains launchpts - this is to safeguard them being swapped...

    // RANDOMLY CHOOSE WHICH d_route TO SWAP TO
    int to_stop = randChoice(drones.second->routes[s.second].size()/*, iteration*/);         // choose stop number to swap out from current d_routes
    Pt* to_node = drones.second->routes[s.second][to_stop];
    if (to_stop == 0 || to_stop > drones.second->routes[s.second].size() - 3) { throw runtime_error("Error: Invalid to_stop chosen"); }
    // ^^ only true if drone route contains launchpts - this is to safeguard them being swapped...

    swap(drones.first->cluster.reefs[findIndexByID(drones.first->routes[s.first][from_stop]->ID, drones.first->cluster.reefs)],
        drones.second->cluster.reefs[findIndexByID(drones.second->routes[s.second][to_stop]->ID, drones.second->cluster.reefs)]);
    swap(drones.first->routes[s.first][from_stop], drones.second->routes[s.second][to_stop]);

    if (swap_print) {
        printf("\nSWAP:\td_routes\tindex\tnode\tReef_id\t\t(x,y)");
        printf("\nc\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.first, from_stop, from_node->ID, from_node->x, from_node->y);
        printf("\nd\t%d\t\t%d\t%d\t(%.1f,%.1f)", s.second, to_stop, to_node->ID, to_node->x, to_node->y);
    }
    return make_pair(from_node->ID, to_node->ID);
}

void match_ms_d_launchPts(MSSoln& msSoln, vector<DroneSoln*>& droneSolns, bool print = false) {
	for (int d = 0; d < droneSolns.size(); d++) {
		if (droneSolns[d]->launchPts.first->ID != msSoln.launchPts[d]->ID) {
			if (print) printf("\n\tMISMATCH LAUNCH PTS: msSoln.launchPts[%d] != droneSolns[%d]->launchPts.first", d, d);
			droneSolns[d]->launchPts.first = msSoln.launchPts[d];
		}
		if (droneSolns[d]->launchPts.second->ID != msSoln.launchPts[d + 1]->ID) {
			if (print) printf("\n\tMISMATCH LAUNCH PTS: msSoln.launchPts[%d] != droneSolns[%d]->launchPts.second", d + 1, d);
			droneSolns[d]->launchPts.second = msSoln.launchPts[d + 1];
		}

        for (auto& route : droneSolns[d]->routes) {
            if (route[0]->ID != droneSolns[d]->launchPts.first->ID) {
				if (print) printf("\n\tMISMATCH LAUNCH PTS: droneSolns[%d]->launchPts.first != droneSolns[%d]->routes[0]", d, d);
				route[0] = droneSolns[d]->launchPts.first;
			}
            if (route[route.size() - 2]->ID != droneSolns[d]->launchPts.second->ID) {
                if (print) printf("\n\tMISMATCH LAUNCH PTS: droneSolns[%d]->launchPts.second != droneSolns[%d]->routes[route.size() - 2]", d, d);
                route[route.size() - 2] = droneSolns[d]->launchPts.second;
            }
            if (route[route.size() - 1]->ID != droneSolns[d]->launchPts.first->ID) {
                if (print) printf("\n\tMISMATCH LAUNCH PTS: droneSolns[%d]->launchPts.first != droneSolns[%d]->routes[route.size() - 1]", d, d);
                route[route.size() - 1] = droneSolns[d]->launchPts.first;
            }
        }
	}
}

/// <summary>
/// /// </summary>
/// <param name="soln">: FullSoln incumbent</param>
/// <param name="iteration"></param>
/// <param name="print">= False</param>
/// <returns></returns>
void OUT_ClusterSwaps(const Problem& inst, FullSoln& soln, bool print = false) {
    if (print) printf("\n---- OUT_Swap ----");
    pair<int, int> c = randSwapChoice(soln.msSoln.clusters.size()/*, iteration*/);    // generate swap pair of clusters
    if (print) printf("\tSwap clusters:\t\t%d\tand\t%d", c.first, c.second);

    pair <int, int> s = random_d_out_Swap(make_pair(soln.droneSolns[c.first], soln.droneSolns[c.second]), print);//,iteration, false);
    swap(soln.msSoln.clusters[c.first]->reefs[findIndexByID(s.first, soln.msSoln.clusters[c.first]->reefs)],
        soln.msSoln.clusters[c.second]->reefs[findIndexByID(s.second, soln.msSoln.clusters[c.second]->reefs)]); // swap clusters

    setLaunchPts(soln.msSoln, inst.weights);//UpdateLaunchPts(soln.msSoln, print);
    match_ms_d_launchPts(soln.msSoln, soln.droneSolns, print); // match launchPts in msSoln with droneSolns
    // Gd update routes before assessing solution
    //PC NOTE: recalculating entire distance matrix every time is very inefficient - most of them won't change, just the ones involving changed launch points
    pair <vector<vector<double>>, vector<vector<double>>>    // dMatrix feeds into routes
        dMatrix = make_pair(soln.droneSolns[c.first]->cluster.getdMatrix(make_pair(soln.msSoln.launchPts[c.first], soln.msSoln.launchPts[c.first + 1])),
            soln.droneSolns[c.second]->cluster.getdMatrix(make_pair(soln.msSoln.launchPts[c.second], soln.msSoln.launchPts[c.second + 1])));
    pair <vector<vector<Pt*>>, vector<vector<Pt*>>>
        routes = make_pair(greedyDroneCluster(*soln.droneSolns[c.first],  dMatrix.first),
            greedyDroneCluster(*soln.droneSolns[c.second], dMatrix.second));

    for (int d = 0; d < soln.droneSolns.size(); d++) {
        if ((soln.msSoln.launchPts[d]->ID != soln.droneSolns[d]->launchPts.first->ID) *
            (soln.msSoln.launchPts[d + 1]->ID != soln.droneSolns[d]->launchPts.second->ID))
            printf("MISMATCH!");
        for (int c = 0; c < soln.droneSolns[d]->routes.size(); c++) {
            if (soln.droneSolns[d]->launchPts.first->ID != soln.droneSolns[d]->routes[c][0]->ID)
                //(new_soln.droneSolns[d]->launchPts.second->ID != new_soln.droneSolns[d]->routes[c][new_soln.droneSolns[d]->routes[route.size() - 2]]->ID))
                printf("MISMATCH!");
        }
    }
    if (print) printf("\t\t-- ^^ OUT_Swap ^^ --");
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/// <summary>
/// SWAP DroneSoln routes directly by ref
/// </summary>
/// <param name="drone"></param>
/// <param name="swap_print"></param>
void random_d_in_Swap(DroneSoln* drone, /*int iteration, */bool swap_print = false) { // d_route = ; tours = d_tours in this cluster
    pair<int, int> s = randSwapChoice(drone->routes.size()/*, iteration*/);   // RANDOMLY CHOOSE WHICH d_routes TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    int from_stop = randChoice(drone->routes[s.first].size()/*, iteration*/);     // randomly choose stop number to swap out from current/chosen d_routes
    if (swap_print) printf("\nSwap From\tindex %d: node = %d\t", from_stop, drone->routes[s.first][from_stop]->ID);
    if (from_stop == 0 || from_stop > drone->routes[s.first].size() - 3) { throw runtime_error("Error: Invalid from_stop chosen"); }

    int to_stop = randChoice(drone->routes[s.second].size()/*, iteration*/);      // randomly choose stop number to swap out from current d_routes
    if (swap_print) printf("\nTo\t\tindex %d: node = %d\t", to_stop, drone->routes[s.second][to_stop]->ID);
    if (to_stop == 0 || to_stop > drone->routes[s.second].size() - 3) { throw runtime_error("Error: Invalid to_stop chosen"); }

    swap(drone->routes[s.first][from_stop], drone->routes[s.second][to_stop]);

    if (swap_print) {
        for (const auto& d_route : drone->routes) {
            printf("\n\t");
            for (const auto& node : d_route) { printf("%d\t", node); }
        }
    }
}

/// <summary>
/// MODIFY Fullsoln soln and return as proposed - mutated cluster_index c to swap within
/// </summary>
/// <param name="soln"></param>
/// <param name="iteration"></param>
/// <param name="print"></param>
/// <returns></returns>
void IN_ClusterSwaps(const Problem& inst, FullSoln& soln, /*int iteration, *//*vector<int> randoms, */bool print = false) {
    vector<vector<Pt*>> routes;
    int c = randChoice(soln.msSoln.clusters.size()/*, iteration*/, true);    // generate random CLUSTER to swap within
    if (print) {
        cout << "\n" << string(50, '~') << "\n";
        printf("\nSwap cluster:\t\t%d\n", c);
        printDroneRoutes(soln.droneSolns[c]);
    }

    random_d_in_Swap(soln.droneSolns[c]/*, iteration*/);          // DONT CREATE COPY, as soln is NOT reference
    if (print) { printDroneRoutes(soln.droneSolns[c]); }

    // these are only created to feed into greedyDroneCluster
    soln.droneSolns[c]->routes = greedyDroneCluster(*soln.droneSolns[c], soln.msSoln.clusters[c]->getdMatrix(make_pair(soln.msSoln.launchPts[c], soln.msSoln.launchPts[c + 1])));                // Run greedy 2-Opt on routes
    if (print) {
        printf("\nOriginal route:\t%.2f\n", soln.getTotalDist(inst.weights));
        for (const auto& drone : soln.droneSolns) { printDroneRoutes(drone); }
    }
}

pair<int, int> random_route_swap(pair<DroneSoln*, DroneSoln*> drones, bool swap_print = false) {//, int iteration) { // d_route = ; tours = d_tours in this cluster
    pair<int, int> d = randSwapChoice(drones.first->routes.size(),
        drones.second->routes.size(), false);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
    // could choose by longest droneRoute

    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", d.first, d.second);

    for (int r = 1; r < drones.first->routes[d.first].size() - 2; r++) { // for each stop in route (not launchPts)
        pair<int, int> reefs_swap_ID = make_pair(
            drones.first->routes[d.first][r]->ID,
            drones.second->routes[d.second][r]->ID);

        //swap reefs in cluster
        swap(drones.first->cluster.reefs[findIndexByID(reefs_swap_ID.first, drones.first->cluster.reefs)],
			drones.second->cluster.reefs[findIndexByID(reefs_swap_ID.second, drones.second->cluster.reefs)]);
        //swap reefs in routes
    	swap(drones.first->routes[d.first][r], drones.second->routes[d.second][r]);
    }

    if (swap_print) {
        printf("\nSWAP:\td_routes\tindex\tnode\tReef_id\t\t(x,y)");
        //printf("\nc\t%d\t\t%d\t%d\t(%.1f,%.1f)", d.first, from_stop, from_node->ID, from_node->x, from_node->y);
        //printf("\nd\t%d\t\t%d\t%d\t(%.1f,%.1f)", d.second, to_stop, to_node->ID, to_node->x, to_node->y);
    }
    return d;
}

void sortieSwap(const Problem& inst, FullSoln& soln, bool print = false) {
    if (print) printf("\n---- SORTIE SWAP ----");
    printf("S");
    pair<int, int> c = randSwapChoice(soln.msSoln.clusters.size());    // cluster pair to swap
    if (print) printf("\tSwap clusters:\t\t%d\tand\t%d", c.first, c.second);
    // swap reefs in DroneSoln routes and clusters. Return d_route indcies
    /*pair <int, int> d = */
    random_route_swap(make_pair(soln.droneSolns[c.first], soln.droneSolns[c.second]), print);

    // update soln.msSoln.clusters with soln.droneSolns[].cluster.reefs for c.first and c.second
    for (int r = 0; r < soln.msSoln.clusters[c.first]->reefs.size(); r++) {
		soln.msSoln.clusters[c.first]->reefs[r] = soln.droneSolns[c.first]->cluster.reefs[r];
		soln.msSoln.clusters[c.second]->reefs[r] = soln.droneSolns[c.second]->cluster.reefs[r];
	}

    setLaunchPts(soln.msSoln, inst.weights);//UpdateLaunchPts(soln.msSoln, print);
    match_ms_d_launchPts(soln.msSoln, soln.droneSolns, print); // match launchPts in msSoln with droneSolns
    // Gd update routes before assessing solution
    //PC NOTE: recalculating entire distance matrix every time is very inefficient - most of them won't change, just the ones involving changed launch points
    pair <vector<vector<double>>, vector<vector<double>>>    // dMatrix feeds into routes
        dMatrix = make_pair(soln.droneSolns[c.first]->cluster.getdMatrix(make_pair(soln.msSoln.launchPts[c.first], soln.msSoln.launchPts[c.first + 1])),
            soln.droneSolns[c.second]->cluster.getdMatrix(make_pair(soln.msSoln.launchPts[c.second], soln.msSoln.launchPts[c.second + 1])));
    pair <vector<vector<Pt*>>, vector<vector<Pt*>>>
        routes = make_pair(greedyDroneCluster(*soln.droneSolns[c.first], dMatrix.first),
            greedyDroneCluster(*soln.droneSolns[c.second], dMatrix.second));

    for (int d = 0; d < soln.droneSolns.size(); d++) {
        if ((soln.msSoln.launchPts[d]->ID != soln.droneSolns[d]->launchPts.first->ID) *
            (soln.msSoln.launchPts[d + 1]->ID != soln.droneSolns[d]->launchPts.second->ID))
            printf("MISMATCH!");
        for (int c = 0; c < soln.droneSolns[d]->routes.size(); c++) {
            if (soln.droneSolns[d]->launchPts.first->ID != soln.droneSolns[d]->routes[c][0]->ID)
                //(new_soln.droneSolns[d]->launchPts.second->ID != new_soln.droneSolns[d]->routes[c][new_soln.droneSolns[d]->routes[route.size() - 2]]->ID))
                printf("MISMATCH!");
        }
    }
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/// <summary>
/// Shell function setting up SA, and calling SA_fn with specified mutator 
/// Randomly swapping by In/Out cluster with 50/50 probability
/// </summary>
/// <param name="soln_init"></param>
/// <param name="in_out"></param>
/// <param name="print_stats"></param>
/// <param name="csv_print"></param>
/// <param name="SA_print"></param>
/// <returns></returns>
FullSoln SwapRandomly(Problem inst, const FullSoln& soln_init, SAparams sa_params, const string& folder_path = "",
    bool print_stats = false, bool csv_print = false, bool csv_update = false) {   //in_out = 1; // 0 = OUT, 1 = IN
    printf("\n\n---------- RANDOM IN/OUT Cluster Swaps - Simulated Annealing ----------\n");
    FullSoln* best_ptr = new FullSoln(soln_init);
    double dist_best = best_ptr->getTotalDist(inst.weights);
    FullSoln* incumbent_ptr = new FullSoln(soln_init);
    FullSoln* proposed_ptr;
    double dist_initial = dist_best;
    double temp = sa_params.initial_temp;           // is this redundant? - temp is updated in SAlog

    srand(42);      // set random seed
    vector<double> sa_new, sa_current, sa_best, sa_temp;
    printf("\nIT\tSW\tTEMP\t\tBEST\t\tINCUMB\t\tPROP\t\tNEW_INCUMB\t\tNEW_BEST");

    for (int iter_num = 0; iter_num < sa_params.num_iterations + 1; ++iter_num) {
        if (best_ptr->msSoln.launchPts.size() == 0) { throw runtime_error("Launch points not set!"); break; }
        bool in_out = rand() % 2;
        bool sortie;
        proposed_ptr = new FullSoln(*incumbent_ptr);
        double dist_incumbent = incumbent_ptr->getTotalDist(inst.weights);
        printf("\n%6d\t%s", iter_num, in_out == 1 ? "I" : "O");
        if (in_out) {
            IN_ClusterSwaps(inst, *proposed_ptr, print_stats);
        }
        else {
            sortie = rand() % 2;
            if (sortie) sortieSwap(inst, *proposed_ptr, print_stats);
			else OUT_ClusterSwaps(inst, *proposed_ptr, print_stats);
        }
        printf("\t%.2e\t%5.2f\t\t%5.2f\t", temp, dist_best, dist_incumbent);

        for (int a = 0; a < proposed_ptr->msSoln.launchPts.size() - 1; a++) {
            if (proposed_ptr->msSoln.launchPts[a]->ID != proposed_ptr->droneSolns[a]->launchPts.first->ID) {
                printf("\n\tMISMATCH LAUNCH PTS: msSoln.launchPts[%d] != droneSolns[%d]->launchPts.first", a, a);
            }
        }
        double dist_proposed = proposed_ptr->getTotalDist(inst.weights);
        printf("\t%5.2f\t", dist_proposed);
        if (accept_new_solution(dist_incumbent, dist_proposed, temp)) {
            // update current incumbent solution with new proposed solution
            delete incumbent_ptr;
            incumbent_ptr = proposed_ptr;       
            dist_incumbent = dist_proposed;
            printf("\t%5.2f\t", dist_incumbent);
            if (dist_proposed < dist_best) {
                if (!in_out && sortie) {
                    printf("");
                }
                // update current best solution with new proposed solution
                delete best_ptr;
                best_ptr = new FullSoln(*proposed_ptr);
                dist_best = dist_proposed;
                printf("\t%5.2f", dist_best);
                if (csv_print && csv_update) {
                    csvUpdate(*best_ptr, inst, in_out, iter_num, sa_params.num_iterations, folder_path);
                }
            }
        }
        else { delete proposed_ptr; }

        temp *= sa_params.cooling_rate;
        sa_new.push_back(dist_proposed);
        sa_current.push_back(dist_incumbent);
        sa_best.push_back(dist_best);
        sa_temp.push_back(temp);
    }
    best_ptr->setSAlog(sa_new, sa_current, sa_best, sa_temp, sa_params);
    if (dist_best == dist_initial) printf("\n\n\tNO IMPROVEMENT MADE\n");
    else printf("\n\n\tBEST\t\t\tINITIAL\t\t\tTEMP\n\t%.3f\t\t%.3f\t\t%.2e", dist_best, dist_initial, temp);
    printf("\n------------- ^^ CLUST_OPT_D_TOURS ^^ --------------\n");
    return *best_ptr;
}
