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
FullSoln BaseSwapRun(FullSoln soln_current, vector<double>& best_dists) {
	//SAparams                  (num_iter, init_temp, cooling_rate)
	int num_iter = 100000;//50000;//25000;//10000;               // fixed at 10000
	double init_temp = 0.2 * soln_current.getTotalDist();
	//double temp_diff = pow(10, -4);
	//double final_temp = init_temp * temp_diff;//pow(10, -5);
	//double cooling_rate = pow((temp_diff), 1 / num_iter);  //0.995;
	double cooling_rate = 0.99975;//0.9995;
	//SAparams sa_params = //SAparams(10000, 0.2 * dist_best, 0.995);
	//	SAparams(num_iter, init_temp, cooling_rate);
	////SAparams(5000, 0.5 * dist_best, 0.98);    //SAparams(1000, 0.5 * dist_best, 0.9);

	//\\//\\//\\//  Randomly run IN/OUT Swaps   //\\//\\//\\//
	FullSoln best = SwapRandomly(soln_current, SAparams(num_iter, init_temp, cooling_rate), print_detail, csv_print);
	printf("\nPrev Dist: \t\t%.2f", best_dists.back());		//printf("\nIn_Swap Dist: \t%.2f", best_in.getTotalDist());
	best_dists.push_back(best.getTotalDist());
	printf("\n\t\tNEW_Swap distance:\t%.2f\n", best_dists.back());

	//\\//\\//\\//  csv print (if solution updated)  //\\//\\//\\//
	//if (csv_print && best_dist.back() != best_dist.at(best_dist.size() - 2)) { 
	csvPrints(best, "FINAL");
	//}
	return best;
}

///////////////////////////////////
//// inst_based

// Function to find factors of a number
vector<int> findFactors(int number, int low=1, int high=INT_MAX) {
	if (high == INT_MAX) high = number;
	vector<int> factors;
	// Loop from 1 to the number
	for (int i = low; i <= high; ++i) { 
		if (number % i == 0) { factors.push_back(i); } 
	}  // If i divides number with 0 remainder, i is a factor of number
	return factors;
}

void VaryInst() {
	return;
}

Problem VaryInstSize(int inst_size) {
	return CreateInst(inst_size);
}

void VaryNumClust(int inst_size) {
	vector<int> factors = findFactors(inst_size);

	for (int f : factors) {
		;
	}

	return;
}
void VaryNumDrones() {
	return;
}
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

