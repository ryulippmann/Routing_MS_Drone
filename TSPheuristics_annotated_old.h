//updated to remove unused functions and classes
#pragma once
#include <cstdio>
#include <string>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cassert>
#include <cfloat>
//#include "TSPmetah.h";

const double gd_tol = 0.000000005;
int gi_itctr;		//counter to count number of iterations

//class CityPair {
//public:
//	int i;
//	int j;
//	double dij;
//
//	CityPair(int ai_i, int ai_j, double ad_dij) : i(ai_i), j(ai_j), dij(ad_dij) {}
//	CityPair() {}
//};//CityPair

class Soln {
public:
	int mi_n;										//number of degrees of freedom in the solution
	double md_z;									//objective value

	Soln(int ai_n) : mi_n(ai_n) {}
	virtual Soln& operator=(const Soln& a_rhs) = 0;	//Note: dynamic cast will be required in override of this
};//Soln

/*Generate TSP instance.
 Because C++ is annoying about returning multiple things, pass *in* lists (by reference) which are filled with the results.
generate_tsp_instance(node_count, all_pairs, distance_matrix, positions)*/
//void g_gen_tsp_instance(const int ai_n, vector<CityPair> &av_dat, vector<vector<double>> &ad_dist, vector<vector<double>> &ad_xy) {
//	
//	// Positions
//	vector<double> d_x(ai_n);
//	vector<double> d_y(ai_n);
//
//	// Resize output lists to match target node count. The distance matrix is a nested list, here.
//	ad_dist.resize(ai_n);
//	ad_xy.resize(ai_n);
//	for (int i = 0; i < ai_n; ++i) {
//		ad_dist[i].resize(ai_n);
//		ad_xy[i].resize(2);
//	}//i
//
//	// Generate random positions in [0,100] interval.
//	for (int i = 0; i < ai_n; ++i) {
//		d_x[i] = rand()*100.0 / RAND_MAX;
//		d_y[i] = rand()*100.0 / RAND_MAX;
//		ad_xy[i][0] = d_x[i];
//		ad_xy[i][1] = d_y[i];
//	}//i
//
//	// More resizing.
//	av_dat.resize(ai_n * ai_n);
//	
//	// Populate the distance list and matrix.
//	int k = 0;
//	for (int i = 0; i < ai_n; ++i) {
//		for (int j = 0; j < ai_n; ++j) {
//			av_dat[k].i = i;
//			av_dat[k].j = j;
//			av_dat[k].dij = sqrt((d_x[i] - d_x[j])*(d_x[i] - d_x[j]) + (d_y[i] - d_y[j])*(d_y[i] - d_y[j]));
//			ad_dist[i][j] = av_dat[k].dij;
//			++k;
//		}//j
//	}//i
//	
//	// No return, modified the passed-by-reference inputs.
//}//g_gen_tsp_instance

// Evaluate objective function: sum up the distances.
// total_distance(node_count, distance_matrix, tour)
double gd_calc_z(int ai_n, const vector<vector<double>> &ad_dist, const vector<int> &ai_tour) {
	double z = ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];

	for (int k = 1; k < ai_n; ++k)
		z += ad_dist[ai_tour[k]][ai_tour[k - 1]];

	return z;
}//gd_calc_z

////generates a random integer between 0 and ai_n - 1
//int gi_rand(int ai_n) {
//	if (ai_n == 0) return 0; // Error check
//	int i = ai_n * (1.0 * rand() / RAND_MAX);
//	i = (i < ai_n ? i : ai_n - 1);	//for the extremely rare occurence of sampling a value of exactly 1
//	return i;
//}//gi_rand

//// Swap two nodes A and B.
//// Also gives the adjusted distance between them, since it's O(1) to calculate when you know the swap and the old length. As opposed to O(n) to recalculate from scratch.
//// swap(node_count, distance_matrix, previous_length, current_tour, swap_A, swap_B)
//double gd_swap(int ai_n, const vector<vector<double>> &ad_dist, double ad_z, vector<int> &ai_tour, int ai_sw1, int ai_sw2) {
//	double z;
//
//	assert(ai_sw1 < ai_sw2); // Error-check.
//	// If I was writing this, it would be "if A<B, swap values and continue",  but it depends on the outer algorithm.
//
//	z = ad_z; // Initial length.
//
//	// Remove the links touching A and B.
//	// If they're adjacent, there are three links -A-B- . Otherwise, four: -A- -B-
//	if (ai_sw1 + 1 == ai_sw2) {
//		// This is why "test ? if_true : if_false" is an awful syntax.
//		
//		// Distance to the first node from the previous.
//		// If it's the first in the list, from the last node.
//		// (C++ vectors don't support passing a nagative index to mean "this many paces from the end".)
//		z -= ai_sw1 > 0 ? ad_dist[ai_tour[ai_sw1 - 1]][ai_tour[ai_sw1]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];
//		// Distance between swapped nodes.
//		z -= ad_dist[ai_tour[ai_sw1]][ai_tour[ai_sw2]];
//		// Same "if the last link is out of range, remove".
//		z -= ai_sw2 < ai_n - 1 ? ad_dist[ai_tour[ai_sw2]][ai_tour[ai_sw2 + 1]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];
//
//		z += ai_sw1 > 0 ? ad_dist[ai_tour[ai_sw1 - 1]][ai_tour[ai_sw2]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[ai_sw2]];
//		z += ad_dist[ai_tour[ai_sw2]][ai_tour[ai_sw1]];
//		z += ai_sw2 < ai_n - 1 ? ad_dist[ai_tour[ai_sw1]][ai_tour[ai_sw2 + 1]] : ad_dist[ai_tour[ai_sw1]][ai_tour[0]];
//	}
//	else if (ai_sw1 + ai_n == ai_sw2 + 1) {
//		// Same as previous edge-case, but first and last in the tour.
//		
//		z -= ad_dist[ai_tour[ai_sw2 - 1]][ai_tour[ai_sw2]];
//		z -= ad_dist[ai_tour[ai_sw2]][ai_tour[ai_sw1]];
//		z -= ad_dist[ai_tour[ai_sw1]][ai_tour[ai_sw1 + 1]];
//
//		z += ad_dist[ai_tour[ai_sw2 - 1]][ai_tour[ai_sw1]];
//		z += ad_dist[ai_tour[ai_sw1]][ai_tour[ai_sw2]];
//		z += ad_dist[ai_tour[ai_sw2]][ai_tour[ai_sw1 + 1]];
//	}//if
//	else {
//		// Same again, for non-adjacent nodes.
//		z -= ai_sw1 > 0 ? ad_dist[ai_tour[ai_sw1 - 1]][ai_tour[ai_sw1]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];
//		z -= ad_dist[ai_tour[ai_sw1]][ai_tour[ai_sw1 + 1]];
//		z -= ad_dist[ai_tour[ai_sw2 - 1]][ai_tour[ai_sw2]];
//		z -= ai_sw2 < ai_n - 1 ? ad_dist[ai_tour[ai_sw2]][ai_tour[ai_sw2 + 1]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];
//
//		z += ad_dist[ai_tour[ai_sw2 - 1]][ai_tour[ai_sw1]];
//		z += ai_sw2 < ai_n - 1 ? ad_dist[ai_tour[ai_sw1]][ai_tour[ai_sw2 + 1]] : ad_dist[ai_tour[ai_sw1]][ai_tour[0]];
//		z += ai_sw1 > 0 ? ad_dist[ai_tour[ai_sw1 - 1]][ai_tour[ai_sw2]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[ai_sw2]];
//		z += ad_dist[ai_tour[ai_sw2]][ai_tour[ai_sw1 + 1]];
//	}//else
//
//	// Actually do the swap.
//	int i_tmp = ai_tour[ai_sw1];
//	ai_tour[ai_sw1] = ai_tour[ai_sw2];
//	ai_tour[ai_sw2] = i_tmp;
//
//	return z;
//}//gd_swap

//NOTE: z calc assume symmetric distance matrix
// Uncrosses links. (Or crosses, if they were uncrossed.)
//		two_opt(node_count,					distance_matrix,	current_length,			current_tour,	link_A_index,	link_b_index,	changed_region)
double gd_two_opt(int ai_n, const vector<vector<double>> &ad_dist, double ad_z, const vector<int> &ai_tour, int ai_hd1, int ai_hd2, vector<int> &ai_nbr/*, bool drones*/) {
	double z;
	int k = ai_hd1;
	int h = ai_hd2 - 1;
	int i_tmp;

	z = ad_z;

	// -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // 
	//RYU HACK:: if considering swapping any zero links...				skip!
	// if either header is from 0 node --> i.e. node - 1 doesn't exist...
	if (ai_hd1 == 0 || ai_hd2 == 0) {
		// AND:: if distance = 0
		if (ad_dist[ai_tour[ai_n - 1]][ai_tour[0]] == 0) {
			//printf("ai_hd1 = %d\tai_hd2 = %d\n", ai_hd1, ai_hd2);
			return z; // BREAK!
			}
		}
	//	if either of the links have 0 distance
	else if (ad_dist[ai_tour[ai_hd1 - 1]][ai_tour[ai_hd1]] == 0 || ad_dist[ai_tour[ai_hd2 - 1]][ai_tour[ai_hd2]] == 0) {
		return z; // BREAK!
	}
	// -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //
	
	// Calculate the new length. Lots of "out of range" guards here, I'd probably write "get distance" helper functions.
	z -= ai_hd1 > 0 ? ad_dist[ai_tour[ai_hd1 - 1]][ai_tour[ai_hd1]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];
	z -= ai_hd2 > 0 ? ad_dist[ai_tour[ai_hd2 - 1]][ai_tour[ai_hd2]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];
	z += ad_dist[ai_tour[ai_hd1]][ai_tour[ai_hd2]];
	z += ad_dist[ai_tour[ai_hd1 > 0 ? ai_hd1 - 1 : ai_n - 1]][ai_tour[ai_hd2 > 0 ? ai_hd2 - 1 : ai_n - 1]];

	// Work inward from both ends, reversing the direction. This might include the beginning/end of the tour.
	ai_nbr = ai_tour;
	if (h < 0)
		h = ai_n - 1;
	while (k != h) {
		i_tmp = ai_nbr[k]; // Three-handed swap: avoid bugs by using temporary storage.
		ai_nbr[k] = ai_nbr[h];
		ai_nbr[h] = i_tmp;

		++k;
		if (k == ai_n) // K reached the end, back to the beginning.
			k = 0;
		if (k != h) {
			--h;
			if (h == -1) // H reached the start, jump to the end.
				h = ai_n - 1;
		}//if
	}//while

	return z;
}//gd_two_opt

////ai_tour is the current tour, ai_nbr should be pre-sized and will be overwritten
////
//// Move a node around in the tour, but only return the changed region.
//// insert(node_count, distance_matrix, current_length, current_tour, remove_index, insert_index, changed_region)
//double gd_insertion(int ai_n, const vector<vector<double>> &ad_dist, double ad_z, const vector<int> &ai_tour, int ai_rmv, int ai_ins, vector<int> &ai_nbr) {
//	double z;
//
//	int i_rmv = ai_rmv;
//	int i_ins = ai_ins;
//
//	int k, j;
//	// Shuffle part of the tour forward/back: find the part to shuffle...
//	if (i_ins < i_rmv) {
//		j = ai_tour[i_rmv];
//		for (k = 0; k <= i_ins; ++k)
//			ai_nbr[k] = ai_tour[k];
//		for (k = i_rmv; k > i_ins + 1; --k)
//			ai_nbr[k] = ai_tour[k - 1];
//		ai_nbr[i_ins + 1] = j;
//		for (k = i_rmv + 1; k < ai_n; ++k)
//			ai_nbr[k] = ai_tour[k];
//	}
//	else {
//		j = ai_tour[i_rmv];
//		for (k = 0; k < i_rmv; ++k)
//			ai_nbr[k] = ai_tour[k];
//		for (k = i_rmv; k < i_ins; ++k)
//			ai_nbr[k] = ai_tour[k + 1];
//		ai_nbr[i_ins] = j;
//		for (k = i_ins + 1; k < ai_n; ++k)
//			ai_nbr[k] = ai_tour[k];
//	}//else
//
//	z = ad_z;
//	z -= i_rmv > 0 ? ad_dist[ai_tour[i_rmv - 1]][ai_tour[i_rmv]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];				//-d_R-1,R
//	z -= i_rmv < ai_n - 1 ? ad_dist[ai_tour[i_rmv]][ai_tour[i_rmv + 1]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];		//-d_R,R+1
//	z -= i_ins < ai_n - 1 ? ad_dist[ai_tour[i_ins]][ai_tour[i_ins + 1]] : ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];		//-d_I,I+1
//	z += i_rmv == 0 ? ad_dist[ai_tour[ai_n - 1]][ai_tour[1]] : 
//		(i_rmv == ai_n - 1 ? ad_dist[ai_tour[ai_n - 2]][ai_tour[0]] : ad_dist[ai_tour[i_rmv - 1]][ai_tour[i_rmv + 1]]);	//+d_R-1,R+1
//	z += ad_dist[ai_tour[i_ins]][ai_tour[i_rmv]];																		//+d_I,R
//	z += i_ins < ai_n - 1 ? ad_dist[ai_tour[i_rmv]][ai_tour[i_ins + 1]] : ad_dist[ai_tour[i_rmv]][ai_tour[0]];			//+d_R,I+1
//
//	return z;
//}//gd_insertion

// ----------------- SNIP ----------------------------

//ai_tour should contain a valid initial solution for the local search
// Full neighbourhood search is useful when many crossed links are expected.
// local_2opt_search(node_count, distance_matrix, tour, use_full_neighbourhood)
// out = pair(obj, route)
pair<double, vector<int>> gd_local_2opt_search(const int& ai_n, const vector<vector<double>> &ad_dist, vector<int> &ai_tour, bool ab_full_nbrhd/*, bool drones=false*/) {
	double z, d_znbr, d_znbr_best;
	vector<int> i_nbr(ai_n);
	int i_hd1, i_hd2, i_hd1_best, i_hd2_best;
	int i_itctr = 0;

	// Initial objective calculation.
	d_znbr_best = gd_calc_z(ai_n, ad_dist, ai_tour);
	// Do-while loop: executes once, tests termination condition at the end.
	do {
		++i_itctr;
		z = d_znbr_best;
		
	//	for (A in range(node_count)):
	//		for (B in range(A,node_count)): // Half the runtime, skips repeats.
		
		for (i_hd1 = 0; i_hd1 < ai_n; ++i_hd1) {
//			for (i_hd2 = 0; i_hd2 < ai_n; ++i_hd2) {
			for (i_hd2 = i_hd1 + 2; i_hd2 < ai_n; ++i_hd2) {
//			if (abs(i_hd1 - i_hd2) > 1 && abs(i_hd1 - i_hd2) < ai_n) {
					// Try cross/uncross.
					// (It's O(1) to check the uncross length.)
			
				d_znbr = gd_two_opt(ai_n, ad_dist, z, ai_tour, i_hd1, i_hd2, i_nbr/*, drones*/);

				// If it helps
				if (d_znbr < d_znbr_best/* && (!drones || !(i_hd1 == 0 && i_hd2 == ad_dist.size()-1))*/) {
					d_znbr_best = d_znbr; // New incumbent.
					if (!ab_full_nbrhd) {
						ai_tour = i_nbr;
						break;	//accept first improvement found
					}//if
					else { // Remember most promising move in neighbourhood.
						i_hd1_best = i_hd1;
						i_hd2_best = i_hd2;
					}//else
				}//if
			}//i_hd2
			if (!ab_full_nbrhd && d_znbr_best < z) // More "break on new incumbent, unless scanning hard".
				break;
		}//i_hd1
		if (ab_full_nbrhd && d_znbr_best < z) { // New incumbent after scanning full neighbourhood.
			d_znbr = gd_two_opt(ai_n, ad_dist, z, ai_tour, i_hd1_best, i_hd2_best, i_nbr/*, drones*/);
			ai_tour = i_nbr;
		}//if
	} while (d_znbr_best + gd_tol < z); // best_neighbour_z + foating-point-tolerance < incumbent_solution
	// Stop when no improvement found.

	gi_itctr = i_itctr; // Pass something out as a global.
	//return z;
	pair<double, vector<int>> gd_out = make_pair(z, ai_tour);
	return gd_out;
}//gd_local_2opt_search

////ai_tour will be cleared/resized
//double gd_nearest_neighbour(int ai_n, vector<vector<double>> &ad_dist, int i_i0, vector<int> &ai_tour) {
//	int i = i_i0;
//	int j;
//	vector<bool> b_visited(ai_n);
//	double d_min;
//	int j_min;
//	double z = 0;
//
//	for (int k = 0; k < ai_n; ++k)
//		b_visited[k] = false;
//
//	ai_tour.resize(ai_n);
//	ai_tour[0] = i;
//	for (int k = 1; k < ai_n; ++k) {
//		b_visited[i] = true;
//		d_min = DBL_MAX;
//		for (j = 0; j < ai_n; ++j) {
//			if (!b_visited[j]) {
//				if (ad_dist[i][j] < d_min) {
//					d_min = ad_dist[i][j];
//					j_min = j;
//				}//if
//			}//if
//		}//j
//		ai_tour[k] = j_min;
//		z += d_min;
//		i = j_min;
//	}//k
//	z += ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];
//	return z;
//}//gd_nearest_neighbour

// //ai_tour will be cleared/resized
//double gd_random_tour(int ai_n, const vector<vector<double>> &ad_dist, vector<int> &ai_tour) {
//	double z;
//
//	vector<bool> b_used(ai_n);
//	for (int i = 0; i < ai_n; ++i)
//		b_used[i] = false;
//
//	ai_tour.resize(ai_n);
//
//	int n = ai_n - 1;
//	int j, k, i_ctr;
//	for (int i = 0; i < ai_n; ++i) {
//		k = gi_rand(n);
//		i_ctr = 0;
//		for (j = 0; j < ai_n; ++j) {
//			if (!b_used[j]) {
//				if (i_ctr == k)
//					break;
//				++i_ctr;
//			}//if
//		}//j
//		b_used[j] = true;
//		ai_tour[i] = j;
//		--n;
//	}//i
//
//	z = gd_calc_z(ai_n, ad_dist, ai_tour);
//
//	return z;
//}//gd_random_tour

class TSPsoln : public Soln {
public:
	vector<int> mi_x; // Tour.

	TSPsoln(int ai_n) : mi_x(ai_n), Soln(ai_n) {}
	TSPsoln(const vector<int> &ai_tour, double ad_z) : mi_x(ai_tour), Soln(ai_tour.size()) {
		md_z = ad_z;
	}
	
	// More reasons to avoid static typing unless 100% needed:
	Soln& operator=(const Soln &a_rhs) {	//instantiation of base class assignment operator
		const TSPsoln *p_rhs = dynamic_cast<const TSPsoln*>(&a_rhs);
		*this = *p_rhs;		//this calls the TSPsoln assignment oeprator below
		return *this;
	}//=

	TSPsoln& operator=(const TSPsoln &a_rhs) {
		if (mi_n != a_rhs.mi_n)
			mi_x.resize(mi_n);
		for (int i = 0; i < mi_n; ++i)
			mi_x[i] = a_rhs.mi_x[i];

		mi_n = a_rhs.mi_n;
		md_z = a_rhs.md_z;

		return *this;
	}//=
};//TSPsoln

//  ----------------- Snipped more class-definitions.

// General remarks:
// Lots of variable names called xx_actual_name, makes it hard to read.
// A few helper functions for "get next in tour, get previous in tour" could help a lot.
// Julia or Python say hi.
// Lots of arguments here which could be read from the vectors.