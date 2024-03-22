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
/// Function for swapping routes
/// </summary>
/// <param name="size_a"></param>
/// <param name="size_b">= -1</param>
/// <param name="duplicate_rule">= True</param>
/// <returns>pair of random indices [0, size_a) and [0, size_b) for swapping routes or stops</returns>
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

/// <summary>
/// return random number [1, size-2] for swapping stops
/// </summary>
/// <param name="size"></param>
/// <param name="clust_choice"></param>
/// <returns></returns>
int randChoice(int size/*, int randomSeed = 12345*/, bool clust_choice = false) {
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
/// <param name="tenders"></param>
/// <param name="iteration"></param>
/// <param name="swap_print"></param>
/// <returns></returns>
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

/// <summary>
/// /// </summary>
/// <param name="soln">: FullSoln incumbent</param>
/// <param name="iteration"></param>
/// <param name="print">= False</param>
/// <returns></returns>
FullSoln OUT_ClusterSwaps(FullSoln soln, int iteration, /*vector<int> randoms, */bool print = false) {
    if (print) printf("---- OUT_Swap ----");
    pair<int, int> c = randSwapChoice(soln.msSoln.clusters.size()/*, iteration*/);    // generate swap pair of clusters
    if (print) printf("\nSwap clusters:\t\t%d\tand\t%d", c.first, c.second);


    pair<TenderSoln, TenderSoln>
        tenders = random_d_out_Swap(make_pair(*soln.tenderSolns[c.first], *soln.tenderSolns[c.second]),
            iteration, false);


    vector<TenderSoln*> tenderSolns;
    for (int i = 0; i < soln.tenderSolns.size(); i++) {
        if (i == c.first) { tenderSolns.push_back(&tenders.first); }      // ****
        else if (i == c.second) { tenderSolns.push_back(&tenders.second); }     // ****
        else { tenderSolns.push_back(soln.tenderSolns[i]); }
    }
    vector<ClusterSoln*> clusters;
    for (int i = 0; i < tenderSolns.size(); i++) {
        clusters.push_back(&(tenderSolns[i]->cluster));
        //if (i == c.first) { clusters.push_back(&tenders.first.cluster); }
        //else if (i == c.second) { clusters.push_back(&tenders.second.cluster); }
                //else { clusters.push_back(soln.msSoln.clusters[i]); 
    }
    //pair <pair<Pt*, Pt*>, pair<Pt*, Pt*>>
    //    launchPts = make_pair(make_pair(soln.msSoln.launchPts[c.first], soln.msSoln.launchPts[c.first + 1]), make_pair(soln.msSoln.launchPts[c.second], soln.msSoln.launchPts[c.second + 1]));
    //pair <vector<vector<double>>, vector<vector<double>>> 
    //    dMatrix = make_pair(tenders.first.cluster.getdMatrix(launchPts.first),
    //                        tenders.second.cluster.getdMatrix(launchPts.second));
    //routes.first = greedyTenderCluster(tenders.first, dMatrix.first);                                 // Run greedy 2-Opt on routes      
    //routes.second = greedyTenderCluster(tenders.second, dMatrix.second);                              // Run greedy 2-Opt on routes 
    //pair<vector<Pt*>, vector<Pt*>> 
    //    reefs = make_pair(  tenders.first.cluster.reefs, 
    //                        tenders.second.cluster.reefs);


    vector<Pt*>
        launchPts = UpdateLaunchPts(clusters/*, true*/);
    for (int d = 0; d < tenderSolns.size(); d++) {
        tenderSolns[d]->launchPts = make_pair(launchPts[d], launchPts[d + 1]);
        for (auto& route : tenderSolns[d]->routes) {
            route[0] = launchPts[d];
            route[route.size() - 2] = launchPts[d + 1];
            route[route.size() - 1] = launchPts[d];
        }
    }

    //// UPDATE tenderSolns with new routes ////
    // dMatrix feeds into routes
    pair <vector<vector<double>>, vector<vector<double>>>
        dMatrix = make_pair(tenderSolns[c.first]->cluster.getdMatrix(make_pair(launchPts[c.first], launchPts[c.first + 1])),
            tenderSolns[c.second]->cluster.getdMatrix(make_pair(launchPts[c.second], launchPts[c.second + 1])));
    // update tenders with new launchPts
    //tenders


    // update routes with Gd before assessing solution
    pair <vector<vector<Pt*>>, vector<vector<Pt*>>>
        routes = make_pair(greedyTenderCluster(tenders.first, dMatrix.first),
            greedyTenderCluster(tenders.second, dMatrix.second));
    // update clusters with new routes
    tenderSolns[c.first]->routes = routes.first;
    tenderSolns[c.second]->routes = routes.second;


    //pair<ClusterSoln, ClusterSoln> 
    //    clusters = make_pair(   ClusterSoln(tenders.first.cluster.reefs),     /*reefs.first*/
                                //                      ClusterSoln(tenders.second.cluster.reefs));   /*reefs.second*/

    MSSoln msSoln = MSSoln(clusters, launchPts);

    FullSoln
        new_soln = FullSoln(msSoln, tenderSolns);          // create new FullSoln with updated clusters and launchPts

    if (print) printf("-- ^^ OUT_Swap ^^ --");
    return new_soln;
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/// <summary>
/// SWAP TenderSoln routes directly by ref
/// </summary>
/// <param name="tender"></param>
/// <param name="swap_print"></param>
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

/// <summary>
/// print tender routes and dists - based on tenderSoln* tender
/// Only called in IN_ClusterSwaps
/// </summary>
/// <param name="tender"></param>
void printTenderRoutes(const TenderSoln* tender) {
	for (const auto& route : tender->routes) {
        for (auto& node : route) {
            printf("\t%d ->", node->ID);
        }
        printf("\nTender route dist:\t%f\n", tender->getTenderRouteDist(route));
    }
}

// MODIFY Fullsoln soln (not ref) and return the swapped copy to save as proposed - mutated cluster_index c to swap within
FullSoln IN_ClusterSwaps(FullSoln soln, int iteration, /*vector<int> randoms, */bool print = false) {
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
            printTenderRoutes(soln.tenderSolns[c]);
        }
        random_d_in_Swap(*soln.tenderSolns[c]/*, iteration*/);          // DONT CREATE COPY, as soln is NOT reference
        if (print) { printTenderRoutes(soln.tenderSolns[c]); }

        // these are only created to feed into greedyTenderCluster
        ClusterSoln* cluster = soln.msSoln.clusters[c];
        pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln.launchPts[c], soln.msSoln.launchPts[c + 1]);
        vector<vector<double>> dMatrix = cluster->getdMatrix(launchPts);
        routes = greedyTenderCluster(*soln.tenderSolns[c], dMatrix);                // Run greedy 2-Opt on routes
        /* PRINT ROUTES AND DISTS FOR EACH SUB - TOUR!! */
    }
    FullSoln new_soln = FullSoln(soln, routes, c);
        if (print) { printf("\nOriginal route:\t%.2f\n", soln.getTotalDist());
            for (const auto& tender : new_soln.tenderSolns) { printTenderRoutes(tender); }
        }
	return new_soln;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/// <summary>
/// Simulated Annealing based on in/out mutator
/// </summary>
/// <param name="initialSolution"></param>
/// <param name="mutator">: FUNCTION taken as IN_ClusterSwaps or OUT_ClusterSwaps</param>
/// <param name="sa_params"></param>
/// <param name="print"></param>
/// <returns>FullSoln best_new</returns>
FullSoln SA_fn(const FullSoln initialSolution,
    function<FullSoln(const FullSoln currentSolution, const int, /*const vector<int>, */const bool)> mutator,
    const SAparams sa_params,/* SAlog& log, */bool print = false) {
    double temp = sa_params.initial_temp;           // is this redundant? - temp is updated in SAlog
    //vector<int> randomness;
    //for (int i = 1; i < sa_params.num_iterations; i++) { randomness.push_back(i); }
    FullSoln best = initialSolution;
    FullSoln incumbent = initialSolution;
    double dist_best = best.getTotalDist();
    double dist_initial = dist_best;
    double dist_incumbent = dist_best;
    double dist_proposed = dist_best;
    printf("\n\tBEST\t\t\tINCUMBENT\t\tTEMP\t\t\tPROPOSED");
    srand(42);      // set random seed
    vector<double> sa_new, sa_current, sa_best, sa_temp;
    //log = SAlog(temp);
    for (int iter_num = 1; iter_num < sa_params.num_iterations + 1; ++iter_num) {
        if (best.msSoln.launchPts.size() == 0) { throw runtime_error("Launch points not set!"); }
        printf("\n%d\t%5.3f\t\t%5.3f\t\t%.2e", iter_num, dist_best, dist_incumbent, temp);
        FullSoln proposed = mutator(incumbent, iter_num, /*randomness, */print);
        dist_incumbent = incumbent.getTotalDist();
        dist_proposed = proposed.getTotalDist();
        printf("\t\t%5.3f", dist_proposed);

        if (accept_new_solution(dist_incumbent, dist_proposed, temp)) {
            incumbent = proposed;       // overwrite old solution, but have been set as const...
            dist_incumbent = incumbent.getTotalDist();
            printf("\tACCEPTED Proposed soln");
            if (dist_proposed < dist_best) {
                best = proposed; 
                dist_best = best.getTotalDist();
                printf("\n\t!!IMPROVED!! Proposed soln\t%.3f\t\t%.3f\t\t%.3f", dist_best, dist_incumbent, dist_proposed);
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
    if (dist_best == dist_initial) printf("\n\n\tNO IMPROVEMENT MADE\n");
    else printf("\n\n\tBEST\t\t\tINITIAL\t\t\tTEMP\n\t%.3f\t\t%.3f\t\t%.2e", dist_best, dist_initial, temp);
    FullSoln best_new = best;           // WHY is this line necessary?!
    return best_new;
}

/// <summary>
/// Shell function setting up SA, and calling SA_fn with specified mutator (IN/OUT)
/// </summary>
/// <param name="soln_prev_best"></param>
/// <param name="in_out"></param>
/// <param name="print_stats">= false</param>
/// <param name="csv_print">= false</param>
/// <param name="SA_print">= true</param>
/// <returns></returns>
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
    int num_iter = 10000;               // fixed at 10000
    double init_temp = 0.2 * dist_best; 
    //double temp_diff = pow(10, -4);
    //double final_temp = init_temp * temp_diff;//pow(10, -5);
    //double cooling_rate = pow((temp_diff), 1 / num_iter);  //0.995;
    double cooling_rate = 0.9995;
    
    SAparams sa_params = //SAparams(10000, 0.2 * dist_best, 0.995);
        SAparams(num_iter, init_temp, cooling_rate);
        //SAparams(5000, 0.5 * dist_best, 0.98);
        //SAparams(1000, 0.5 * dist_best, 0.9);
    if (in_out == 0) {      //best = OUT_ClusterSwaps(soln_prev_best, 0, true);
        best = SA_fn(soln_prev_best, OUT_ClusterSwaps, sa_params/*, log*/); 
        printf("\n^^ OUT SWAPS ^^\n");
    }
    else {                  //best = IN_ClusterSwaps(soln_prev_best, 0, true);
        best = SA_fn(soln_prev_best, IN_ClusterSwaps, sa_params/*, log*//*, true*/);
        printf("\n^^ IN SWAPS ^^\n");
    }

    printf("\n------------- ^^ CLUST_OPT_D_TOURS ^^ --------------\n");
    return best;
}

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
FullSoln SwapRandomly(const FullSoln soln_prev_best, 
    //int num_iterations = 10000, double initial_temperature = 200, double cooling_rate = 0.999,
    bool print_stats = false, bool csv_print = false, bool SA_print = true) {   //in_out = 1; // 0 = OUT, 1 = IN
    printf("\n\nRANDOM IN/OUT Cluster Swaps!\n---------- CLUST_OPT_D_TOURS - Simulated Annealing ----------\n");

    //SAlog log = SAlog(/*initial_temperature*/);
    FullSoln best(soln_prev_best);
    double dist_best = best.getTotalDist();
    FullSoln incumbent = soln_prev_best;
    double dist_initial = dist_best;

    //SAparams                  (num_iter, init_temp, cooling_rate)
    int num_iter = 10000;               // fixed at 10000
    double init_temp = 0.2 * dist_best;
    //double temp_diff = pow(10, -4);
    //double final_temp = init_temp * temp_diff;//pow(10, -5);
    //double cooling_rate = pow((temp_diff), 1 / num_iter);  //0.995;
    double cooling_rate = 0.9995;
    SAparams sa_params = //SAparams(10000, 0.2 * dist_best, 0.995);
        SAparams(num_iter, init_temp, cooling_rate);
    //SAparams(5000, 0.5 * dist_best, 0.98);    //SAparams(1000, 0.5 * dist_best, 0.9);
    double temp = sa_params.initial_temp;           // is this redundant? - temp is updated in SAlog
    
    printf("\n\tBEST\t\t\tTEMP\t\t\tPROPOSED\t\tINCUMBENT");
    srand(42);      // set random seed
    vector<double> sa_new, sa_current, sa_best, sa_temp;
    //log = SAlog(temp);
        int in_swaps = 0;
        int out_swaps = 0;
    for (int iter_num = 1; iter_num < sa_params.num_iterations + 1; ++iter_num) {
        if (best.msSoln.launchPts.size() == 0) { throw runtime_error("Launch points not set!"); break; }
        FullSoln proposed = incumbent;
        // randomly set mutator as IN or OUT
        bool in_out = rand() % 2;
        if (in_out == 0) {      //best = OUT_ClusterSwaps(soln_prev_best, 0, true);
            proposed = OUT_ClusterSwaps(incumbent, iter_num, /*randomness, */print_stats);
            //printf("\tOUT SWAPS\n");
            out_swaps += 1;
		}
		else {                  //best = IN_ClusterSwaps(soln_prev_best, 0, true);
            proposed = IN_ClusterSwaps(incumbent, iter_num, /*randomness, */print_stats);
            //printf("\tIN SWAPS\n");
            in_swaps += 1;
		}

        double dist_proposed = proposed.getTotalDist(), dist_incumbent = incumbent.getTotalDist();
        printf("\n%d\t%5.3f\t\t%.2e\t\t%5.3f\t\t%5.3f\t%s", iter_num, dist_best, temp, dist_proposed, dist_incumbent, in_out == 1 ? "IN" : "OUT");
        if (accept_new_solution(dist_incumbent, dist_proposed, temp)) {
            incumbent = proposed;       // overwrite old solution, but have been set as const...
            dist_incumbent = incumbent.getTotalDist();
            printf("\t\tACCEPTED Proposed soln");
            if (dist_proposed < dist_best) {
                best = proposed;
                dist_best = best.getTotalDist();
                printf("\n\t!!IMPROVED!! Proposed soln\t\t%.3f", dist_best);
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
    if (dist_best == dist_initial) printf("\n\n\tNO IMPROVEMENT MADE\n");
    else printf("\n\n\tBEST\t\t\tINITIAL\t\t\tTEMP\n\t%.3f\t\t%.3f\t\t%.2e", dist_best, dist_initial, temp);
    FullSoln best_new = best;           // WHY is this line necessary?!

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