#pragma once


void sensitivity() {
	//sensitivity analysis
	//for (int i = 0; i < noClust; ++i) {
	//    for (int j = 0; j < noDrones; ++j) {
	//        for (int k = 0; k < dCap; ++k) {
	//            //cout << "i=" << i << " j=" << j << " k=" << k << endl;
	//            //cout << "i=" << i << " j=" << j << " k=" << k << " " << noClust * noDrones * dCap << endl;
	//            //cout << "i=" << i << " j=" << j << " k=" << k << " " << no_pts << endl;
	//            //cout << "i=" << i << " j=" << j << " k=" << k << " " << noClust * noDrones * dCap << " " << no_pts << endl;
	//            //cout << "i=" << i << " j=" << j << " k=" << k << " " << noClust * noDrones * dCap << " " << no_pts << " " << noClust * noDrones * dCap - no_pts << endl;
	//            //cout << "i=" << i << " j=" << j << " k=" << k << " " << noClust * noDrones * dCap << " " << no_pts << " " << noClust * noDrones * dCap - no_pts << " " << noClust * noDrones * dCap / no_pts << endl;
	//            //cout << "i=" << i << " j=" << j << " k=" << k << " " << noClust * noDrones * dCap << " " << no_pts << " " << noClust * noDrones * dCap - no_pts << " " << noClust * noDrones * dCap / no_pts << " " << noClust * noDrones * dCap % no_pts << endl;
	//            //cout << "i=" << i << " j=" << j << " k=" << k << " " << noClust * noDrones * dCap << " " << no_pts << " "
}

///////////////////////////////////

/// <summary>
///
/// </summary>
/// <param name="soln_current"></param>
/// <param name="best_dists"></param>
/// <returns></returns>
FullSoln BaseSwapRun(Problem inst, FullSoln soln_current, vector<double>& best_dists, int run_iteration, const string& folder_path ="") {
	//SAparams                  (num_iter, init_temp, cooling_rate)
	int num_iter = 1000;		//10000;		// 100000;		// 5000;	// 
	double init_temp =	0.2 * soln_current.getTotalDist(inst.weights);
	//double temp_diff = pow(10, -4);
	//double final_temp = init_temp * temp_diff;//pow(10, -5);
	//double cooling_rate = pow((temp_diff), 1 / num_iter);  //0.995;
	double cooling_rate = 0.99;	//0.999;//  0.9999;		// 0.998;	// 

	//\\//\\//\\//  Randomly run IN/OUT Swaps   //\\//\\//\\//
	FullSoln best = SwapRandomly(inst, soln_current, SAparams(num_iter, init_temp, cooling_rate), folder_path+"/"+to_string(run_iteration),
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
vector<FullSoln> FullRun(const int& iter, const Problem& inst, const string& batch="", const string& sens_run_name = "") {
	//\\//\\//\\//\\  ClusterSoln Construction  //\\//\\//\\//
	vector<ClusterSoln*> clusters = kMeansConstrained(inst.kMeansIters, inst.getReefPointers(), inst.ms.cap);
	string folder_path;
	if (print_detail) printClusters(clusters);		// PRINT clusters //

	if (csv_print) {
		if (batch == "") folder_path = createFolder(inst.time, sens_run_name);
		else folder_path = createFolder(batch, sens_run_name);
	}

	//\\//\\//\\//\\//  MsSoln Construction //\\//\\//\\//\\//
	MSSoln msSoln(clusters, inst.ms);				// No launchPts initialised yet
	vector<pair<double, MSSoln>> msSolns = initMsSoln(clusters, msSoln, inst.weights, print_detail);

	//\\//\\//\\//\\  DroneSoln Construction  \\//\\//\\//\\/
	vector<DroneSoln> droneSolns = initDroneSoln(inst, msSoln, print_detail);
	vector<DroneSoln*> ptr_droneSolns;
	for (const auto& soln : droneSolns) { ptr_droneSolns.push_back(new DroneSoln(soln)); } // Assuming DroneSoln has a copy constructor

	//\\//\\//\\//\\/  FullSoln Construction  /\\//\\//\\//\\/
	FullSoln full_init(msSoln, ptr_droneSolns);
	printf("Full Soln Dist:\t%.2f", full_init.getTotalDist(inst.weights, print_detail));
	if (csv_print) {
		csvPrintStops(inst, batch, "reef_set");
		csvPrints(full_init, inst, "INIT", iter, folder_path);
	}
	////////////////////////////////
	vector<FullSoln> fullSolns;
	fullSolns.push_back(full_init);
	vector<double> best_dist{ fullSolns.back().getTotalDist(inst.weights) };			// initialise best_dist as vector with best solution distance	

	fullSolns.push_back(
		BaseSwapRun(inst, fullSolns.back(), best_dist, iter, folder_path)
	);

	return fullSolns;
}

///////////////////////////////////
//// inst_based

vector<int> Factors(int num) {
	vector<int> factors;
	for (int i = 2; i <= num/2; ++i) {
		if (num % i == 0) {
			factors.push_back(i);
		}	// If i divides num with 0 remainder, i is a factor of num
	}
	return factors;
}

/// <summary>
/// given number of pts and dCap
/// </summary>
/// <returns>
/// vector of pairs: (number of clusters and drones, and total_dist of solution)
/// </returns>
vector <pair < pair<int, int>, pair<double, FullSoln> >> VaryNum_clustXdrone(const Problem& inst) {
	vector<vector<FullSoln>> fullSolns;
	vector<pair<int,int>> factors_pairs;
	vector <pair < pair<int, int>, pair<double, FullSoln> >> results;

	int total_drone_routes = inst.reefs.size() / inst.get_dCap();
	vector<int> factors = Factors(total_drone_routes);

	for (int f : factors) {
		factors_pairs.push_back(make_pair(f, total_drone_routes / f));
	}
	for (int i = 0; i < factors_pairs.size(); i++) {
		int num_clust = factors_pairs[i].first;
		int num_drone = factors_pairs[i].second;
		Problem sens_inst = CreateInst(inst, num_clust, num_drone, inst.get_dCap());
		fullSolns.push_back(FullRun(i, sens_inst, inst.time, "varyClusts"));
	}
	for (int i = 0; i < fullSolns.size(); i++) {
		double dist = fullSolns[i].back().getTotalDist(inst.weights);
		results.push_back(make_pair(factors_pairs[i], make_pair(dist, fullSolns[i].back())));
	}
	return results;
}

///////////////////////////////////

/// <summary>
/// given number of pts and num_clust
/// </summary>
/// <returns>
/// vector of pairs: (dCap and number of drones, and total_dist of solution)
/// </returns>
vector <pair < pair<int, int>, pair<double, FullSoln> >> VaryDrones(const Problem& inst) {
	vector<vector<FullSoln>> fullSolns;
	vector<pair<int, int>> factors_pairs;
	vector <pair < pair<int, int>, pair<double, FullSoln> >> results;

	int pts_in_clust = inst.reefs.size() / inst.getnumClusters();
	vector<int> factors = Factors(pts_in_clust);

	for (int f : factors) {
		factors_pairs.push_back(make_pair(f, pts_in_clust / f));
	}
	for (int i = 0; i < factors.size(); i++) {
		int dCap = factors_pairs[i].first;
		int num_drone = factors_pairs[i].second;
		Problem sens_inst = CreateInst(inst, inst.getnumClusters(), num_drone, dCap);
		fullSolns.push_back(FullRun(i, sens_inst, inst.time, "varyDroneCap"));
	}
	for (int i = 0; i < fullSolns.size(); i++) {
		double dist = fullSolns[i].back().getTotalDist(inst.weights);
		results.push_back(make_pair(factors_pairs[i], make_pair(dist, fullSolns[i].back())));
	}
	return results;
}

///////////////////////////////////

/// <summary>
/// vary weights by a sens_incr times between min and max values
/// </summary>
/// <param name="w_ms"></param>
/// <param name="w_d"></param>
/// <param name="sens_incr"></param> number of variation iterations below & above base weights
/// <returns>
/// results of varying weights as vector of pairs: (ms weight and d weight, and total_dist of solution)
/// </returns>
vector <pair < pair<double, double>, pair<double, FullSoln> >> VaryWeights(pair<double, double> bounds_w_ms, pair<double, double> bounds_w_d, const Problem& inst, int sens_incr=10) {
	vector<vector<FullSoln>> fullSolns;
	vector<pair<double, double>> weighting_pairs;
	vector <pair < pair<double, double>, pair<double, FullSoln> >> results;

	// perform sensitivty analysis based on more acceptable range of values
	double var_w_ms = (bounds_w_ms.second - bounds_w_ms.first)/(sens_incr-1);
	double var_w_d = (bounds_w_d.second - bounds_w_d.first)/(sens_incr-1);

	for (double ms = bounds_w_ms.first; ms <= bounds_w_ms.second; ms += var_w_ms) {
		for (double d = bounds_w_d.first; d <= bounds_w_d.second; d += var_w_d) {
			weighting_pairs.push_back(make_pair(static_cast<double>(ms), static_cast<double>(d)));
		}
	}
	for (int i = 0; i < weighting_pairs.size(); i++) {
		Problem sens_inst = CreateInst(inst, weighting_pairs[i]);
		fullSolns.push_back(FullRun(i, sens_inst, inst.time, "sens_weights"));
	}
	for (int i = 0; i < fullSolns.size(); i++) {
		double dist = fullSolns[i].back().getTotalDist(inst.weights);
		results.push_back(make_pair(weighting_pairs[i], make_pair(dist, fullSolns[i].back())));
	}
	return results;
}

///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
///////////////////////////////////

//Problem VaryInstSize(int inst_size) {return CreateInst(inst_size);}

//void VaryClusterBunching() {return;}

///////////////////////////////////
//// clust_based

//void VarySwapIters() {

//	//FullSoln best = SwapRandomly(fullSolns.back(), print_detail, csv_print);
//	//printf("\nPrev Dist: \t\t%.2f", best_dist.back());		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
//	//best_dist.push_back(best.getTotalDist());
//	//printf("\n\t\tNEW_Swap distance:\t%.2f\n", best_dist.back());
//	//// vv fullSolns is not creating new fullSoln objects, but rather just pointing to the same object
//	//fullSolns.push_back(best);
//	//// csv print if solution updated
//	//if (csv_print && best_dist.back() != best_dist.at(best_dist.size() - 2)) { csvPrints(best, "FINAL"); }
//	return;
//}

//void VarySwapTypeProb() {return;}

