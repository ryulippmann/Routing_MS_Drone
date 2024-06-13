#pragma once

///////////////////////////////////

/// <summary>
/// </summary>
/// <param name="inst"></param>
/// <param name="soln_current"></param>
/// <param name="best_dists"></param>
/// <param name="run_iteration"></param>
/// <param name="folder_path"></param>
/// <param name="sa_it_cr"> = pair(iterations, cooling rate)</param>
/// <returns>FullSoln best</returns>
FullSoln SwapShell(const Problem& inst, const FullSoln& soln_current, vector<double>& best_dists, int run_iteration, const string& folder_path = "", pair<int, double> sa_it_cr = make_pair(0, 0)) {
	//SAparams                  (num_iter, init_temp, cooling_rate)
	double init_temp = 0.2 * soln_current.getTotalDist(inst.weights);
	if (sa_it_cr == make_pair(0, 0)) { sa_it_cr = make_pair(3 * pow(10, 4), 0.9995)/*8 * pow(10, 4), 0.9998)*//*5 * pow(10, 4), 0.9997)*/; }	//pow(10, 4), 0.999); }		//pow(10,3), 0.99); }	// 	   (pow(10,5), 0.9999); }	//

	//\\//\\//\\//  Randomly run IN/OUT Swaps   //\\//\\//\\//
	FullSoln best = SwapRandomly(inst, soln_current, SAparams(sa_it_cr.first, init_temp, sa_it_cr.second), folder_path + "/" + to_string(run_iteration),
		print_detail, csv_print);
	printf("\nPrev Dist: \t\t%.2f", best_dists.back());		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
	best_dists.push_back(best.getTotalDist(inst.weights));
	printf("\n\t\tNEW_Swap distance:\t%.2f\n", best_dists.back());

	//\\//\\//\\//  csv print (if solution updated)  //\\//\\//\\//
	if (csv_print) { csvPrints(best, inst, "FINAL", run_iteration, folder_path); }
	return best;
}

/// <summary>
/// 
/// </summary>
/// <param name="iter"></param>
/// <param name="INST"></param> GLOBAL VARIABLE included to allow for sensitivity changes
/// <returns></returns>
vector<FullSoln> FullRun(const int& iter, const Problem& inst, const string& batch = "", pair<int, double> sa_param_it_cr = make_pair(0, 0)) {
	string folder_path;
	if (csv_print) {
		if (batch == "") folder_path = createFolder(inst.time);
		else folder_path = createFolder(batch);
	}
	auto start_time = chrono::high_resolution_clock::now();  // Start timing

	//\\//\\//\\//\\  ClusterSoln Construction  //\\//\\//\\//
	vector<ClusterSoln*> clusters = kMeansConstrained(inst.kMeansIters, inst.getReefPointers(), inst.ms.cap);
	if (print_detail) printClusters(clusters);		// PRINT clusters //

	//\\//\\//\\//\\//  MsSoln Construction //\\//\\//\\//\\//
	MSSoln msSoln(clusters, inst.ms);				// No launchPts initialised yet
	vector<pair<double, MSSoln>> msSolns = initMsSoln(clusters, msSoln, inst.weights, print_detail);

	//\\//\\//\\//\\  DroneSoln Construction  \\//\\//\\//\\/
	vector<DroneSoln> droneSolns = initDroneSoln(inst, msSoln, print_detail);
	//vector<DroneSoln*> ptr_droneSolns;
	//for (const auto& soln : droneSolns) { ptr_droneSolns.push_back(new DroneSoln(soln)); } // Assuming DroneSoln has a copy constructor

	//\\//\\//\\//\\/  FullSoln Construction  /\\//\\//\\//\\/
	FullSoln full_init(msSoln, droneSolns);
	printf("Full Soln Dist:\t%.2f", full_init.getTotalDist(inst.weights, print_detail));
	if (csv_print) {
		csvPrintStops(inst, folder_path, "reef_set");
		csvPrints(full_init, inst, "INIT", iter, folder_path);
	}
	////////////////////////////////
	vector<FullSoln> fullSolns;
	fullSolns.push_back(full_init);
	vector<double> best_dist{ fullSolns.back().getTotalDist(inst.weights) };			// initialise best_dist as vector with best solution distance	

	fullSolns.push_back(
		SwapShell(inst, fullSolns.back(), best_dist, iter, folder_path, sa_param_it_cr)
	);

	auto end_time = chrono::high_resolution_clock::now();
	chrono::duration<double> elapsed = end_time - start_time;
	writeRuntimeToFile(folder_path + "/runtime", iter, elapsed);
	//printf("Run time: %d minutes %.2f seconds\n", static_cast<int>(elapsed.count()) / 60, fmod(elapsed.count(), 60));

	return fullSolns;
}

///////////////////////////////////

vector<int> Factors(const int& num) {
	vector<int> factors;
	for (int i = 2; i <= num / 2; ++i) {
		if (num % i == 0) {
			factors.push_back(i);
		}	// If i divides num with 0 remainder, i is a factor of num
	}
	return factors;
}

vector<pair<int, int>> GenerateFactorPairs(const int& num) {
	vector<int> factors = Factors(num);
	vector<pair<int, int>> factor_pairs;
	for (int f : factors) {
		factor_pairs.push_back(make_pair(f, num / f));
	}
	return factor_pairs;
}

vector <pair < pair<int, int>, pair<double, FullSoln> >>	sensitivityResults(const Problem& inst, const vector<vector<FullSoln>>& fullSolns, const vector<pair<int, int>>& factors_pairs) {
	vector <pair < pair<int, int>, pair<double, FullSoln> >> results;
	for (int i = 0; i < fullSolns.size(); i++) {
		double dist = fullSolns[i].back().getTotalDist(inst.weights);
		results.push_back(make_pair(factors_pairs[i], make_pair(dist, fullSolns[i].back())));
	}
	return results;
}
vector <pair < pair<double, double>, pair<double, FullSoln> >>	sensitivityResults(const Problem& inst, const vector<vector<FullSoln>>& fullSolns, const vector<pair<double, double>>& weight_pairs) {
	vector <pair < pair<double, double>, pair<double, FullSoln> >> results;
	for (int i = 0; i < fullSolns.size(); i++) {
		double dist = fullSolns[i].back().getTotalDist(inst.weights);
		results.push_back(make_pair(weight_pairs[i], make_pair(dist, fullSolns[i].back())));
	}
	return results;
}

///////////////////////////////////
//// clust_based
/// <summary>
/// given number of pts and dCap
/// </summary>
/// <returns>
/// vector of pairs: (number of clusters and drones, and total_dist of solution)
/// </returns>
vector <pair < pair<int, int>, pair<double, FullSoln> >> VaryClusters(const Problem& inst, string& sens_run, int runs = 1) {
	sens_run = "_vary_Clusts";
	int total_drone_routes = inst.reefs.size() / inst.get_dCap();
	vector<vector<FullSoln>> fullSolns;
	vector<pair<int, int>> factors_pairs = GenerateFactorPairs(total_drone_routes);

	for (int i = 0; i < factors_pairs.size(); i++) {
		int num_clust = factors_pairs[i].first;
		int num_drone = factors_pairs[i].second;
		for (int j = 0; j < runs; j++) {
			Problem sens_inst = CreateInst(inst, num_clust, num_drone, inst.get_dCap());
			fullSolns.push_back(FullRun(i, sens_inst, inst.time + sens_run, make_pair(pow(10, 3), 0.99)));
		}
	}
	return sensitivityResults(inst, fullSolns, factors_pairs);
}

///////////////////////////////////

//// inst_based
/// <summary>
/// given number of pts and num_clust
/// </summary>
/// <returns>
/// vector of pairs: (dCap and number of drones, and total_dist of solution)
/// </returns>
vector <pair < pair<int, int>, pair<double, FullSoln> >> VaryDrones(const Problem& inst, string& sens_run, int runs = 1) {
	sens_run = "_vary_dCap";
	int pts_in_clust = inst.reefs.size() / inst.getnumClusters();
	vector<vector<FullSoln>> fullSolns;
	vector<pair<int, int>> factors_pairs = GenerateFactorPairs(pts_in_clust);

	for (int i = 0; i < factors_pairs.size(); i++) {
		int dCap = factors_pairs[i].first;
		int num_drone = factors_pairs[i].second;
		for (int j = 0; j < runs; j++) {
			Problem sens_inst = CreateInst(inst, inst.getnumClusters(), num_drone, dCap);
			fullSolns.push_back(FullRun(i, sens_inst, inst.time + sens_run, make_pair(pow(10, 3), 0.99)));
		}
	}
	return sensitivityResults(inst, fullSolns, factors_pairs);
}

///////////////////////////////////

/// <summary>
/// vary weight ratio by a sens_incr times between min and max values
/// </summary>
/// <param name="inst"></param>
/// <param name="sens_run"></param>
/// <param name="bounds_ratio"></param>
/// <param name="sens_incr"></param>
/// <returns></returns>
vector <pair < pair<double, double>, pair<double, FullSoln> >> VaryWeights(const Problem& inst, string& sens_run, const pair<double, double>& bounds_ratio, int sens_incr = 10, int runs = 1) {
	// update weighting pair to int's to match other sensitivity weightings? Just un-normalised...
	sens_run = "_vary_weights";
	vector<vector<FullSoln>> fullSolns;
	vector<pair<double, double>> weighting_pairs;

	// perform sensitivty analysis based on more acceptable range of values
	double var_ratio = abs(bounds_ratio.second - bounds_ratio.first) / (sens_incr - 1);

	for (double r = bounds_ratio.first; r <= bounds_ratio.second; r += var_ratio) {
		double ms = r, d = 1;
		weighting_pairs.push_back(make_pair(ms, d));
	}
	for (int i = 0; i < weighting_pairs.size(); i++) {
		for (int j = 0; j < runs; j++) {
			Problem sens_inst = CreateInst(inst, weighting_pairs[i]);
			fullSolns.push_back(FullRun(i, sens_inst, inst.time + sens_run, make_pair(pow(10, 4), 0.999)));
		}
	}
	return sensitivityResults(inst, fullSolns, weighting_pairs);
}

///////////////////////////////////
