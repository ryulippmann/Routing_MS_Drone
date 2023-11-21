#pragma once
//#include <vector>
//#include <iostream>
#include <cmath>
using namespace std;

double sum(vector<double>& vec_to_sum) {
    double sum_of_elems = 0.0;
    for (const auto& n : vec_to_sum) {
        sum_of_elems += n;
    }
    return sum_of_elems;
}//sum
