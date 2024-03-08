#pragma once
#include <chrono>
#include <ctime>
#include <sstream>

////////////////////////////// PRINTS //////////////////////////////

void csvPrintStops(const vector<ClusterSoln*>& clusters, const string& file_name) {
    ofstream outputFile(file_name + ".csv");    // create .csv file from string name
    if (outputFile.is_open()) {
        outputFile << "ClusterID" << "," << "PtID" << "," << "X" << "," << "Y" << "\n";
        for (const auto& cluster : clusters) {
            for (const auto& point : cluster->reefs) {
                outputFile << cluster->ID << "," << point->ID << "," << point->x << "," << point->y << "\n";
            }
        }
        outputFile.close();
        cout << "\nReef stop points saved to: " << file_name << ".csv\n";
    }
    else {
        cerr << "Failed to open the output file.\n";
    }
}
//void csvPrintStops(vector<Pt> reefs, string file_name) {
//    ofstream outputFile(file_name + ".csv");    // create .csv file from string name
//    if (outputFile.is_open()) {
//        outputFile << "ID" << "," << "X" << "," << "Y" << "," << "Cluster" << "," << "Visited" << "\n";
//        for (const auto& point : reefs) {
//            outputFile << point.ID << "," << point.x << "," << point.y << "," << point.cluster << "," << point.visited << "\n";
//        }
//        outputFile.close();
//        cout << "\nReef stop Points saved to: " << file_name << ".csv\n";
//    }
//    else { cerr << "!! PRINT STOPS: Failed to open the output file.\n"; }
//    return;
//}

void csvPrintRoutes(vector<vector<Pt*>> routes, string file_name, string extension = "") {     // Print the routes to a CSV file
    auto now = chrono::system_clock::now();
    time_t time = chrono::system_clock::to_time_t(now);
    tm localTime;                       // Convert time to local time

    localtime_s(&localTime, &time);
    stringstream ss;                    // Create a string stream to format the date and time
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%y-%m-%d_%H-%M-%S ", &localTime);
    file_name = buffer + file_name + " - " + extension;     // file_name = "drone_routes/" + string(buffer) + file_name;
    file_name = "drone_routes/" + file_name;

    ofstream outputFile(file_name + ".csv");   // create .csv file from string name
    if (outputFile.is_open()) {
        outputFile << "\n";                                 // skip header row
        for (const auto& route : routes) {                  // for each route in routes
            for (const auto& stop : route) {                // for each stop in route
                outputFile << stop->ID << ","/* << stop->x << "," << stop->y << ","*/;                  // output each stop across each row
            }
            outputFile << "\n";                             // go to next row after all stops in route have been output
        }
        outputFile.close();
        cout << "\nRoute points saved to: " << file_name << ".csv\n";
    }
    else { cerr << "!! PRINT ROUTES: Failed to open the output file.\n"; }
    return;
}

void csvPrintMSRoutes(vector<Pt*> launch_route, string file_name) {     // Print the routes to a CSV file
    auto now = chrono::system_clock::now();
    time_t time = chrono::system_clock::to_time_t(now);
    // Convert time to local time
    tm localTime;

    localtime_s(&localTime, &time);
    // Create a string stream to format the date and time
    stringstream ss;
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%y-%m-%d_%H-%M-%S ", &localTime);
    file_name = buffer + file_name;
    file_name = "ms_routes/" + file_name;

    ofstream outputFile(file_name + ".csv");   // create .csv file from string name
    if (outputFile.is_open()) {
        outputFile << "\n";                                 // skip header row
        for (const auto& stop : launch_route) {                // for each stop in route
                outputFile << stop->x << "," << stop->y << "\n";                  // output each stop across each row
        }
        outputFile << "\n";                             // go to next row after all stops in route have been output

        outputFile.close();
        cout << "\nMS_launchpt coordinates saved to: " << file_name << ".csv\n";
    }
    else { cerr << "!! PRINT ROUTES: Failed to open the output file.\n"; }
    return;
}

//double printRouteStats(vector<Vehicle>& vehicles, int capacity, string hr_name = "", double ex_dist = 0,
//    bool printstats = false) {
//    // PRINT: Sub-tour dists, and total dist
//    double route_dist = 0;
//    if (printstats) {
//        cout << "\n" << "//" << string(41, '=') << "\t" << hr_name << "\t" << string(43, '=')
//            << "\\\\\n||\tBest Route : ";
//    }
//    for (const auto& vehicle : vehicles) {
//        if (printstats) { printf("\t%.2f", vehicle.route_dist); }
//        route_dist += vehicle.route_dist;
//        //route_dists.push_back(vehicle.route_dist);
//    }//for vehicles
//    if (printstats) { cout << string(2, '\t') << "   ||"; }
//    double improvement = ex_dist - route_dist;
//
//    if (printstats) {
//        if (improvement != 0) {
//            cout << "\n||\tBest Distance:\t" << route_dist << string(11, '\t') << "   ||\n"; //<< sum(route_dists) 
//            if (ex_dist != 0) {
//                printf("||\tImprovement:\t%5.2f", improvement);
//                cout << string(11, '\t') << "   ||\n";
//            }
//            cout << "||\tVehicle ID\tReefs\t";
//            for (int i = 1; i <= capacity; i++) { printf("%d\t", i); }       //"1\t2\t3\t4\t5\t6\t7\t8\t9\t10\t"); }
//            printf("1  || \n");
//            cout << "||" << string(113, '-') << "||\n";
//        }
//        else { cout << "\n||\tNo improvement :(" << string(11, '\t') << "   ||\n"; }
//        for (const auto& v : vehicles) {
//            if (v.reef_stops.size() == 0) { break; }
//            printf("||%2d\t\t\t", v.id);
//            for (const auto& r : v.reef_stops) {
//                printf("\t%2d", r);
//            }
//            cout << " ||\n";
//        }
//        cout << "\\\\" << string(113, '=') << "//\n";
//    }
//    //cout << "Reef ID\t\tVehicle\n";
//    //for (const auto& r : reefs) {
//    //    printf("%d\t\t%d\n", r.id, r.visited);
//    //}
//    return route_dist;
//}
//
//void printMSroute(const Mothership& mothership, const vector<ClusterPoint>& centroids) {
//    printf("\n Cluster order\t  x\t\t  y\n");
//    for (const auto& stop : mothership.launch_stops) {
//        printf(" %d\t\t %2.2f\t\t %2.2f\n", stop, centroids[stop].x, centroids[stop].y);
//    } 
//    cout << "\n" << string(50, '-') << "\n";
//    return;
//}

string csvPrintSA(vector<double> solution_best_dist, vector<double> solution_current_dist, vector<double> solution_new_dist, 
    vector<double> solution_temp, int initial_temperature, double cooling_rate, 
    int num_iterations, string file_name, string c = NULL) {
    auto now = chrono::system_clock::now();
    time_t time = chrono::system_clock::to_time_t(now);
    tm localTime;                                   // Convert time to local time

    localtime_s(&localTime, &time);
    stringstream ss;                                    // Create a string stream to format the date and time
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%y-%m-%d_%H-%M-%S ", &localTime);
    file_name = buffer + file_name + '-' + c;
    file_name = "sa_plots/" + file_name;
    cout << "\n" << string(30, '*') << "\nSaving points to: " << file_name << ".csv\n" << string(30, '.') << "\n";

    ofstream outputFile(file_name + ".csv");   // create .csv file from string name
    outputFile << "Best_dist,Current_dist,New_dist,Temp,,initial_temperature," << initial_temperature
        << ",,cooling_rate," << cooling_rate << ",,best_soln," << solution_best_dist.back()
        << ",,iter," << num_iterations << "\n";                                 // skip header row
    if (outputFile.is_open()) {
        for (int i = 0; i < solution_best_dist.size() - 1;i++) {
            outputFile << solution_best_dist[i] << ","          // output each solution
                << solution_current_dist[i] << ","
                << solution_new_dist[i] << ","
                << solution_temp[i] << "\n";
        }
        outputFile << "\n";                             // go to next row after all stops in route have been output
        outputFile.close();
        cout << "SA Points saved to: " << file_name << ".csv\n" << string(30, '*') << "\n";
    }
    else { cerr << "!! PRINT SA SOLNS: Failed to open the output file.\n"; }
    return file_name;
}

void csvPrintSA_Time(string file_name, chrono::seconds elapsed_time) {
    int minutes = elapsed_time.count() / 60;
    int seconds = elapsed_time.count() % 60;
    //file_name = "sa_plots/" + file_name;
    ofstream outputFile(file_name + ".csv", ios::app);   // create .csv file from string name
    if (outputFile.is_open()) {
		outputFile << ",,,,Time," << minutes << ",m, " << seconds << ",s\n";                                 // skip header row
		outputFile.close();
		cout << "Time saved to: " << file_name << ".csv\n" << string(30, '*') << "\n";
	} else { cerr << "!! PRINT SA SOLNS: Failed to open the output file.\n"; }
    return;
}

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

void csvPrints(FullSoln best_new, bool in_out) {
    csvPrintStops(best_new.msSoln.clusters, "reef_stops");
    for (const auto& vehicle : best_new.tenderSolns) {
		csvPrintRoutes(vehicle->routes, "drone_route_list", boolToString(in_out));
	}
    //csvPrintRoutes(best_new.tenderSolns, "drone_routes", "best");)
    csvPrintMSRoutes(best_new.msSoln.launchPts, "ms_launch_route");
    csvPrintSA(best_new.sa_log, best_new.sa_params, "sa_log", "best");
    //csvPrintSA_Time(filename_SA, fn_elapsed_time);
    return;
}