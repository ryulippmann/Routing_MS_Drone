#pragma once
//#include <chrono>
//#include <ctime>
//#include <sstream>

#include <fstream>

//#include <iostream>
//#include <random>
//#include <vector>
using namespace std;
//comment out when linked...


struct SAparams {
    int initial_temp;
    double cooling_rate;
    int num_iterations;
    string file_name;
    //string c = NULL;
    SAparams(int num_iterations, int initial_temp, double cooling_rate) :
        num_iterations(num_iterations), initial_temp(initial_temp), cooling_rate(cooling_rate) {};
};

struct SAlog {
    vector<double> best_dist;
    vector<double> current_dist;
    vector<double> new_dist;
    vector<double> temp;
    SAlog(double new_dist_val, double current_dist_val, double best_dist_val, double temp_val) {
        new_dist.push_back(new_dist_val);
        current_dist.push_back(current_dist_val);
        best_dist.push_back(best_dist_val);
        temp.push_back(temp_val);
    }
    SAlog(double temp_val) {
        temp.push_back(temp_val);
    }
};

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

// return pair of random numbers [0, size_a) and [0, size_b) for swapping routes or stops
pair<int, int> randSwapChoice(int size_a, int randomSeed = 12345, int size_b = -1, bool within_clust = true) {       // RANDOMLY CHOOSE WHICH d_route TO SWAP
    //random_device rd;
    mt19937 gen_route(randomSeed/*rd()*/);                          //swap between random d_routes
    uniform_int_distribution<int> dist_a(0, size_a - 1);
    if (size_b == -1) { size_b = size_a; }
    uniform_int_distribution<int> dist_b(0, size_b - 1);

    int d_route_a;// = dist(gen);   // initialise swap out FROM current d_routes
    int d_route_b;// = dist(gen);   // initialise swap out TO   current d_routes
    do {
        d_route_a = dist_a(gen_route);   // choose route to swap out FROM current d_routes
        d_route_b = dist_b(gen_route);   // choose route to swap out TO   current d_routes
    } while (within_clust && d_route_a == d_route_b);   // if same routes chosen
    return make_pair(d_route_a, d_route_b);
}

// return random number [1, size-2] for swapping stops
int randSwapStopChoice(int size, int randomSeed = 12345) {
    //random_device rd;
    mt19937 gen_stop(randomSeed);                      // check d_route to ensure d_route.size-2 is the right size distribution
    uniform_int_distribution<int> dist(1, size - 3);     //!constrict swaps between launch/retrieval nodes - i.e. only reef nodes
    //int from_stop = dist_from(gen_from_stop);   // d_route stop [1,10]    // choose stop number to swap out from current d_routes   
    return dist(gen_stop);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

pair<vector<vector<int>>, double> random_d_in_Swap(TenderSoln* tender, const vector<vector<double>> dMatrix, int iteration, bool swap_print = false) { // d_route = ; tours = d_tours in this cluster

    // check if d_routes > 2

    /* RANDOMLY CHOOSE WHICH d_route TO SWAP*/
    pair<int, int> s = randSwapChoice(tender.routes.size(), iteration);   // RANDOMLY CHOOSE WHICH d_route TO SWAP
    if (swap_print) printf("\nSwap between d_routes:\t%d\tand\t%d", s.first, s.second);

    /* RANDOMLY CHOOSE WHICH STOP IN CHOSEN d_routes TO SWAP FROM*/
    int from_stop = randSwapStopChoice(tender.routes[s.first].size(), iteration);     // choose stop number to swap out from current d_routes
    //dist_from(gen_from_stop);   // d_route stop [1,10]    // choose stop number to swap out from current d_routes
    if (swap_print) printf("\nSwap From\tindex %d: node = %d\t", from_stop, d_routes[s.first][from_stop]);

    /* RANDOMLY CHOOSE WHICH STOP IN CHOSEN d_routes TO SWAP TO*/
    int to_stop = randSwapStopChoice(d_routes[s.second].size(), iteration);      // choose stop number to swap out from current d_routes
    //dist_to(gen_to_stop);   // d_route stop [1,10]    // choose stop number to swap out from current d_routes
    if (swap_print) printf("\nTo\t\tindex %d: node = %d\t", to_stop, d_routes[s.second][to_stop]);

    swap(d_routes[s.first][from_stop], d_routes[s.second][to_stop]);

    if (swap_print) {
        for (const auto& d_route : d_routes) {
            printf("\n\t");
            for (const auto& node : d_route) { cout << node << "\t"; }
        }
    } //cout << "\n";
    return make_pair(d_routes, (d_routes, dMatrix));
}

FullSoln IN_ClusterSwaps(FullSoln soln,  int iteration, bool print = false) {
    vector<ClusterSoln*> clusters = soln.msSoln.clustSolns;
    int c = randSwapChoice(clusters.size(), iteration).first;    // generate swap pair of routes
    pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln.launchPts[c], soln.msSoln.launchPts[c + 1]);
    pair<vector<vector<int>>, double> temp_route = random_d_in_Swap(soln.tenderSolns[c], clusters[c]->getdMatrix(c, launchPts), iteration); //, cluster.routes);
    TenderSoln new_clust(clusters[c], launchPts);            //drone_cluster_routes[c] =
    // transform temp_route.first to vector<vector<Pt*>> routes

    for (int d = 0; d < temp_route.first.size(); d++) {   // UPDATE ROUTES incl -1 and -2 nodes!
        new_clust.routes[d].push_back(0);
        for (int s = 0; s < temp_route.first[d].size(); s++) {
            new_clust.routes[d].push_back(temp_route.first[d][s] + 1);//new_clust.routes[d].reef_stops.push_back(new_cluster.routes[d][s]);
        }
    }
    
    new_clust.routes = temp_route.first;        // vector<vector<Pt*>> <- vector<vector<int>> routes
    new_clust.route_dist = temp_route.second;
    new_clust = droneWithinClusterGreedy(clusters[soln.clustOrder.first[c + 1] - 1]);       ///*vector<Reef_pt>& reefs, */
    clusters[soln.clustOrder.first[c + 1] - 1] = new_clust;

    /* PRINT ROUTES AND DISTS FOR EACH SUB - TOUR!! */
    if (print) {
        printf("\nOriginal route:\t%d\n", soln.getTotalDist());
        for (const auto& tender : soln.tenderSolns) { 
            for (const auto& node : tender->routes) {
                printf("\t%d", node); 
            } printf("\n"); 
        }
    }
    return soln;
}

/////////////////////////////////////////////////////////////////////////

void random_d_out_Swap(Cluster* cluster_a, Cluster* cluster_b, int iteration,
    bool swap_print = false) { // d_route = ; tours = d_tours in this cluster

    /* RANDOMLY CHOOSE WHICH d_route TO SWAP */
    pair<int, int> s = randSwapChoice(cluster_a.routes.size(), iteration, cluster_b.routes.size(), false);   // RANDOMLY CHOOSE WHICH d_route TO SWAP

    /* RANDOMLY CHOOSE WHICH d_route TO SWAP FROM*/
    int from_stop = randSwapStopChoice(cluster_a.routes[s.first], iteration);        // choose stop number to swap out from current d_routes
    int from_node = cluster_a.routes[s.first][from_stop] - 1;
    /* RANDOMLY CHOOSE WHICH d_route TO SWAP TO*/
    int to_stop = randSwapStopChoice(cluster_b.routes[s.second], iteration);         // choose stop number to swap out from current d_routes
    int to_node = cluster_b.routes[s.second][to_stop] - 1;
    if (swap_print) {
        printf("\nSWAP:\td_routes\tindex\tnode\tReef_id\t\t(x,y)");
        printf("\nc\t%d\t\t%d\t%d\t%d\t(%.1f,%.1f)", s.first, from_stop, from_node, cluster_a.reef_objects[from_node].id, cluster_a.reef_objects[from_node].x, cluster_a.reef_objects[from_node].y);
        printf("\nd\t%d\t\t%d\t%d\t%d\t(%.1f,%.1f)", s.second, to_stop, to_node, cluster_b.reef_objects[to_node].id, cluster_b.reef_objects[to_node].x, cluster_b.reef_objects[to_node].y);
    }
    swap(cluster_a.reef_objects[from_node], cluster_b.reef_objects[to_node]);

    clearClusterData(cluster_a);
    clearClusterData(cluster_b);

    return;
}

FullSoln OUT_ClusterSwaps(FullSoln soln, int iteration, bool print = false) {
    if (print) printf("---- OUT_Swap ----");
    vector<Cluster*> clusters = soln.msSoln->clustSoln->clusters;
    pair<int, int> c = randSwapChoice(clusters.size(), iteration);    // generate swap pair of routes
    if (print) printf("\nSwap clusters:\t\t%d\tand\t%d", c.first, c.second);
    //pair< vector<vector<int>>, vector<vector<int>> > swapped_routes = make_pair(clusters[c].routes, clusters[d].routes); //ms_route.first;
    random_d_out_Swap(soln.msSoln->clustSoln->clusters[c.first], soln.msSoln->clustSoln->clusters[c.second], iteration/*clusters[c].routes, clusters[c].dMatrix, clusters[d].routes, clusters[d].dMatrix*/); //, cluster.routes);

    // DO THIS OUTSIDE THIS FUNCTION?!
    // RE-CLUSTER
    //pair<vector<vector<Reef_pt>>, vector<ClusterPoint> > cc = clusterAndCentroid(reefs, numClusters, noDrones, dCap, clusters, false, false);
    /*vector<vector<Reef_pt>> clusteredPoints = */clusterAndCentroid(soln/*problem, solution*/, false, false);                                 //vector<ClusterPoint> centroids = cc.second;
    //for (int c = 0; c < clusteredPoints.size(); c++) {
    //    solution.clusters[c].reef_objects = clusteredPoints[c];
    //}
    //vector<pair<vector<vector<int>>, double>> drone_cluster_routes(problem.numClusters);
    // /////////////////////////////////////////////////////////////////
    /**** ADD LAUNCH AND RETRIEVAL NODES TO CLUSTERS ****/
    for (int a = 1; a < clusters.size() + 1; a++) {
        pair<Pt*, Pt*> launchPts = make_pair(soln.msSoln->launchPts[a], soln.msSoln->launchPts[a + 1]);
        ////Add drop off and pick up pts to each cluster...
        TenderSoln new_clustTendersoln(clusters[a], droneWithinClusterNearestNeighbour(soln.msSoln, a),
            launchPts);		//vector<vector<Pt*>> cluster_routes = droneWithinClusterNearestNeighbour(&msSoln, c);		
        //Cluster* new_cluster;
        //new_cluster = droneWithinClusterNearestNeighbour(        // INITIALISE SOLUTION = Nearest Neighbour for each cluster...
        //    clusters[solution.clustOrder.first[a] - 1]);
        for (int d = 0; d < new_clustTendersoln.routes.size(); d++) {   // UPDATE ROUTES incl -1 and -2 nodes!
            //new_cluster.drones[d].reef_stops.push_back(0/*-1*/);
            for (int s = 0; s < new_clustTendersoln.routes[d].size(); s++) {
                ;//new_clustTendersoln.routes[d].reef_stops.push_back(new_cluster.routes[d][s]);
            }
        }
        new_clustTendersoln.routes = greedyTenderCluster(&new_clustTendersoln, clusters[a]->getdMatrix(a, launchPts));     // IMPROVE SOLUTION = Greedy 2-Opt for each cluster...
        //Cluster* cluster, vector<vector<Pt*>> routes, pair<Pt*, Pt*> launchPts
        //soln.tenderSolns[a] = TenderSoln(new_clustTendersoln;
    }

    if (print) printf("-- ^^ OUT_Swap ^^ --");
    return soln;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

FullSoln SA(const FullSoln initialSolution, 
    function<FullSoln(const FullSoln currentlSolution, const int, const bool)> mutator, 
    const SAparams sa_params, SAlog& log) {
    FullSoln incumbent = initialSolution;
    FullSoln best = initialSolution;
    double temp = sa_params.initial_temp;
    bool print = true;

    //vector<Cluster> best_clusters = initialSolution.clusters;
    //vector<Cluster> incumbent_clusters = best_clusters;
    //vector<Cluster> proposed_clusters;                               // new/candidate/incumbent clusters to be updated if accepted

    for (int iter_num = 1; iter_num < sa_params.num_iterations; ++iter_num) {
        FullSoln proposed = mutator(initialSolution, iter_num, print);

        if (accept_new_solution(incumbent.getTotalDist(), proposed.getTotalDist(), temp)/*rand() / static_cast<double>(RAND_MAX) < exp((incumbent.getTotalDist() - proposed.getTotalDist()) / temperature)*/) {
            // overwrite old solution, but have been set as const...`            
            incumbent = proposed;//FullSoln(proposed.msSoln, proposed.tenderSolns);//FullSoln(proposed.clusters, proposed.mothership, proposed.routes, proposed.clustOrder);
            if (proposed.getTotalDist() < best.getTotalDist()) {
                best = proposed;//Solution(proposed.clusters, proposed.mothership, proposed.routes, proposed.clustOrder); //proposed;
            }
        }
        temp *= sa_params.cooling_rate;
    }
    return best;
}

string SwapFunction(//Problem problem, Solution solution, /*vector<Cluster>& clusters, const pair<vector<int>, double> clustOrder, *//*Mothership& mothership, */
    FullSoln gd_soln, bool in_out, int num_iterations = 10000,/*1000000,*/ int initial_temperature = 200, double cooling_rate = 0.999,
    bool print_stats = false, bool csv_print = false, bool SA_print = true) {             // Print stats for debugging
    //default_random_engine generator;                           // Create a random number generator engine
    //uniform_real_distribution<double> distribution(0.0, 1.0);  // Create a uniform real distribution between 0.0 and 1.0
    ////in_out = 1; // 0 = OUT, 1 = IN
    if (in_out == 0) { printf("\n\nWithOUT Cluster\n"); }
    else if (in_out == 1) { printf("\n\nWithIN Cluster\n"); }
    else { throw runtime_error("Error: Invalid in_out argument provided"); }
    printf("---------- CLUST_OPT_D_TOURS - Simulated Annealing ----------\n");

    // vector of results for SA plot
    SAparams sa_params = SAparams(num_iterations, initial_temperature, cooling_rate);

    SAlog log = SAlog(initial_temperature);

    FullSoln best = gd_soln;
    if (in_out == 0) {
        best = SA(gd_soln, OUT_ClusterSwaps, sa_params, log);
    }
    //else {
    //    best = SA(gd_soln, IN_ClusterSwaps, sa_params, log);
    //}

    //clusters = best_clusters;
    string filename_SA;
    if (SA_print) {
        //cout << "\nCurrent route:\t"    << log.current_dist.back() << "\n"; //cout << calculate_drone_distance(current_route, cluster.dMatrix) << "\t\t\t\t";        
        //for (const auto& cluster_routes : current_routes) {
        //    for (const auto& route : cluster_routes) {
        //        for (const auto& node : route) { cout << "\t" << node; } cout << "\n";
        //    } cout << "\n";
        //}        
        cout << "Best route:\t" << log.best_dist.back() << "\n";
        for (const auto& tender : best.tenderSolns) {
            for (const auto& route : tender->routes) {
                for (const auto& node : route) { 
                    cout << "\t" << node; } 
                cout << "\n";
            } 
            cout << "\n";
        }

        if (in_out == 0) filename_SA = csvPrintSA(//log.solution_best_dist, log.solution_current_dist, log.solution_new_dist, log.solution_temp,
            //initial_temperature, cooling_rate, num_iterations, 
            log, sa_params, "d_opt_stops-SA_plot", "out");
        else filename_SA = csvPrintSA(//log.solution_best_dist, log.solution_current_dist, log.solution_new_dist, log.solution_temp,
            //initial_temperature, cooling_rate, num_iterations, 
            log, sa_params, "d_opt_stops-SA_plot", "in");
    }
    cout << "\n------------- ^^ CLUST_OPT_D_TOURS ^^ --------------\n";
    return filename_SA/*routes*/;
}//    double gd_2opt_dist;
