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
FullSoln BaseSwapRun(Problem inst, FullSoln soln_current, vector<double>& best_dists, int run_iteration) {
	//SAparams                  (num_iter, init_temp, cooling_rate)
	int num_iter = 1000;//50000;//100000;//25000;               // fixed at 10000
	double init_temp = 0.2 * soln_current.getTotalDist();
	//double temp_diff = pow(10, -4);
	//double final_temp = init_temp * temp_diff;//pow(10, -5);
	//double cooling_rate = pow((temp_diff), 1 / num_iter);  //0.995;
	double cooling_rate = 0.9995;//0.99975;//

	//\\//\\//\\//  Randomly run IN/OUT Swaps   //\\//\\//\\//
	FullSoln best = SwapRandomly(inst, soln_current, SAparams(num_iter, init_temp, cooling_rate), run_iteration, 
		print_detail, csv_print);
	printf("\nPrev Dist: \t\t%.2f", best_dists.back());		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
	best_dists.push_back(best.getTotalDist());
	printf("\n\t\tNEW_Swap distance:\t%.2f\n", best_dists.back());

	//\\//\\//\\//  csv print (if solution updated)  //\\//\\//\\//
	if (csv_print /*&& best_dists.back() != best_dists.at(best_dists.size() - 2)*/) { 
	csvPrints(best, "FINAL", run_iteration);
	}
	return best;
}

vector<FullSoln> FullRun(const int& iter, Problem inst) {
	//\\//\\//\\//\\  ClusterSoln Construction  //\\//\\//\\//
	vector<ClusterSoln*> clusters = kMeansConstrained(inst.kMeansIters, inst.getReefPointers(), inst.ms.cap);
	//int maxIterations = inst.kMeansIters;
	//vector<Pt*> points = inst.getReefPointers();
	//const int numClusters = inst.ms.cap;

	if (print_detail) printClusters(clusters);		// PRINT clusters //

	if (csv_print) {
		createFolder();
		createRunFolder(iter);
	}
	//if (csv_print) csvPrintClusters(clusters, "clusters_init");		// CSV PRINT clusters //
	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\	
	//\\//\\//\\//\\  ClusterSoln Initialised \\//\\//\\//\\//

	//\\//\\//\\//\\//  MsSoln Construction //\\//\\//\\//\\//
	MSSoln msSoln(clusters);				// No launchPts initialised yet
	vector<pair<double, MSSoln>> msSolns = initMsSoln(clusters, msSoln, print_detail);
	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//\\//\\//\\//\\/   MsSoln Initialised   /\\//\\//\\//\\//

	//\\//\\//\\//\\  DroneSoln Construction  \\//\\//\\//\\/
	vector<DroneSoln> droneSolns = initDroneSoln(inst, msSoln, print_detail);
	vector<DroneSoln*> ptr_droneSolns;
	for (const auto& soln : droneSolns) { ptr_droneSolns.push_back(new DroneSoln(soln)); } // Assuming DroneSoln has a copy constructor
	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//\\//\\//\\//\   DroneSoln Initialised   \//\\//\\//\\//		

	//\\//\\//\\//\\/  FullSoln Construction  /\\//\\//\\//\\/
	FullSoln full_init(msSoln, ptr_droneSolns);
	printf("Full Soln Dist:\t%.2f", full_init.getTotalDist(print_detail));
	if (csv_print) {
		csvPrintStops("reef_set");
		csvPrints(full_init, "INIT", iter);
	}
	////////////////////////////////
	vector<FullSoln> fullSolns;
	fullSolns.push_back(full_init);
	vector<double> best_dist{ fullSolns.back().getTotalDist() };			// initialise best_dist as vector with best solution distance	
	//\\    \\//    //\\    \\//    //\\    \\//    //\\    \\
	//\\//\\//\\//\\   FullSoln Initialised   \\//\\//\\//\\//		

	fullSolns.push_back(
		BaseSwapRun(inst, fullSolns.back(), best_dist, iter)
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

vector <pair < pair<int, int>, pair<double, FullSoln> >> VaryNum_droneXclust() {
	vector<vector<FullSoln>> fullSolns;
	int total_drone_routes = inst.reefs.size()/inst.get_dCap();
	vector<int> factors = Factors(total_drone_routes);
	vector<pair<int,int>> factors_pairs;
	for (int f : factors) {
		factors_pairs.push_back(make_pair(f, total_drone_routes / f));
	}
	for (int i = 0; i < factors_pairs.size(); i++) {
		int num_clust = factors_pairs[i].first;
		int num_drone = factors_pairs[i].second;
		Problem sens_inst = CreateInst(inst, num_clust, num_drone);
		fullSolns.push_back(FullRun(i, sens_inst));
	}
	//vector<double> dists;
	//for (int i = 0; i < fullSolns.size(); i++) {
	//	dists.push_back(fullSolns[i].back().getTotalDist());
	//}
	vector <pair < pair<int, int>, pair<double, FullSoln> >> results;
	for (int i = 0; i < fullSolns.size(); i++) {
		results.push_back(make_pair(factors_pairs[i], make_pair(fullSolns[i].back().getTotalDist(), fullSolns[i].back())));
	}
	return results;
}

///////////////////////////////////
///////////////////////////////////

Problem VaryInstSize(int inst_size) {
	return CreateInst(inst_size);
}

void VaryInst() {
	// call varyNum_droneXclust() for each instance size
	return;
}

///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
///////////////////////////////////

void VaryDroneCap() {
	return;
}

///////////////////////////////////

void VaryClusterBunching() {
	return;
}

///////////////////////////////////
//// clust_based

void VarySwapIters() {
	//FullSoln best = SwapRandomly(fullSolns.back(), print_detail, csv_print);
	//printf("\nPrev Dist: \t\t%.2f", best_dist.back());		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
	//best_dist.push_back(best.getTotalDist());
	//printf("\n\t\tNEW_Swap distance:\t%.2f\n", best_dist.back());
	//// vv fullSolns is not creating new fullSoln objects, but rather just pointing to the same object
	//fullSolns.push_back(best);
	//// csv print if solution updated
	//if (csv_print && best_dist.back() != best_dist.at(best_dist.size() - 2)) { csvPrints(best, "FINAL"); }

	return;
}

void VarySwapTypeProb() {
	return;
}

void VarykMeansIters() {
	return;
}

///////////////////////////////////

void VaryWeights() {
	return;
}

