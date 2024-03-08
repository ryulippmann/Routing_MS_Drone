#pragma once
//#include <string>
//#include <vector>

struct SAparams {
    SAparams(int num_iterations, double initial_temp, double cooling_rate) :
        num_iterations(num_iterations), initial_temp(initial_temp), cooling_rate(cooling_rate) {};

    int num_iterations;
    double initial_temp;
    double cooling_rate;
    string file_name;
    //string c = NULL;
};

struct SAlog {
    SAlog() {}
    SAlog(double new_dist_val, double current_dist_val, double best_dist_val, double temp_val) {
        new_dist.push_back(new_dist_val);
        current_dist.push_back(current_dist_val);
        best_dist.push_back(best_dist_val);
        temp.push_back(temp_val);
    }
    SAlog(double temp_val) {
        temp.push_back(temp_val);
    }

    vector<double> best_dist;
    vector<double> current_dist;
    vector<double> new_dist;
    vector<double> temp;
};
