#pragma once
//called in mothership.h
#include <cstdio>
#include <string>
//#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cassert>
#include <cfloat>
//#include "TSPmetah.h";

const double gd_tol = 0.000000005;
int gi_itctr;		//counter to count number of iterations

class Soln {
public:
	int mi_n;										//number of degrees of freedom in the solution
	double md_z;									//objective value

	Soln(int ai_n) : mi_n(ai_n) {}
	virtual Soln& operator=(const Soln& a_rhs) = 0;	//Note: dynamic cast will be required in override of this
};//Soln

// Evaluate objective function: sum up the distances.
// total_distance(node_count, distance_matrix, tour)
double gd_calc_z(int ai_n, const vector<vector<double>>& ad_dist, const vector<int>& ai_tour) {
	double z = 0;//ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];
	//for (int k = 0; k < ai_n - 2; ++k)
	for (int k = 0; k < ai_n - 1; ++k)
			z += ad_dist[ai_tour[k]][ai_tour[k + 1]];
			//ad_dist[ai_tour[k]][ai_tour[k - 1]];
	z += ad_dist[ai_tour[ai_n - 1]][ai_tour[0]];
	return z;
}//gd_calc_z

//NOTE: z calc assume symmetric distance matrix
// Uncrosses links. (Or crosses, if they were uncrossed.)
//		two_opt(node_count,					distance_matrix,	current_length,			current_tour,	link_A_index,	link_b_index,	changed_region)
double gd_two_opt(int ai_n, const vector<vector<double>>& ad_dist, double ad_z, const vector<int>& ai_tour, int ai_hd1, int ai_hd2, vector<int>& ai_nbr/*, bool drones*/) {
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

/// <summary>
/// ai_tour should contain a valid initial solution for the local search
/// Full neighbourhood search is useful when many crossed links are expected.
/// local_2opt_search(node_count, distance_matrix, tour, use_full_neighbourhood)
/// out = pair(obj, route)
/// </summary>
/// <param name="ai_n"></param>
/// <param name="ad_dist"></param>
/// <param name="ai_tour"></param>
/// <param name="ab_full_nbrhd"></param>
/// <returns></returns>
pair<double, vector<int>> gd_local_2opt_search(const int& ai_n, const vector<vector<double>>& ad_dist, vector<int> ai_tour, bool ab_full_nbrhd/*, bool drones=false*/) {
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

class TSPsoln : public Soln {
public:
	vector<int> mi_x; // Tour.

	TSPsoln(int ai_n) : mi_x(ai_n), Soln(ai_n) {}
	TSPsoln(const vector<int>& ai_tour, double ad_z) : mi_x(ai_tour), Soln(ai_tour.size()) {
		md_z = ad_z;
	}

	// More reasons to avoid static typing unless 100% needed:
	Soln& operator=(const Soln& a_rhs) {	//instantiation of base class assignment operator
		const TSPsoln* p_rhs = dynamic_cast<const TSPsoln*>(&a_rhs);
		*this = *p_rhs;		//this calls the TSPsoln assignment oeprator below
		return *this;
	}//=

	TSPsoln& operator=(const TSPsoln& a_rhs) {
		if (mi_n != a_rhs.mi_n)
			mi_x.resize(mi_n);
		for (int i = 0; i < mi_n; ++i)
			mi_x[i] = a_rhs.mi_x[i];

		mi_n = a_rhs.mi_n;
		md_z = a_rhs.md_z;

		return *this;
	}//=
};//TSPsoln

// General remarks:
// Lots of variable names called xx_actual_name, makes it hard to read.
// A few helper functions for "get next in tour, get previous in tour" could help a lot.
// Julia or Python say hi.
// Lots of arguments here which could be read from the vectors.