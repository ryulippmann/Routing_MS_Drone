#pragma once
#include <sstream>
#include <fstream>

#ifdef _WIN32
#include <direct.h>   // For mkdir on Windows
#endif

bool directoryExists(const string& path) {
    struct stat info;
    return stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

bool createDirectory(const string& path) {
#ifdef _WIN32
    return _mkdir(path.c_str()) == 0;
#else
    return mkdir(path.c_str(), 0777) == 0; // Note: 0777 sets full access permissions
#endif
}

string inFolder(const string& in_folder = "") {
	string folderPath = "outputs/" + inst.time;    // Define the path of the folder
	if (!in_folder.empty()) folderPath += "/" + in_folder;
	return folderPath;
}

string createFolder(const string& folder_name = "", const string& sub_folder ="") {
    string folderPath = "outputs/" + inst.time;    // Define the path of the folder
    if (!folder_name.empty()) 
        folderPath += "/" + folder_name;
        if (!sub_folder.empty())
            folderPath += "/" + sub_folder;

    if (directoryExists(folderPath)) {    // Check if the folder already exists
        cout << "Folder '" << folderPath << "' already exists." << endl;
        return folderPath;
    }
    // Create the folder
    if (createDirectory(folderPath)) { cout << "Folder '" << folderPath << "' created successfully." << endl; }
    else { cerr << "Failed to create folder '" << folderPath << "'." << endl; }
}

/*void */string createRunFolder(int iter) {
    string folderPath = "outputs/" + inst.time + "/" + to_string(iter);    // Define the path of the folder
    if (directoryExists(folderPath)) {    // Check if the folder already exists
        cout << "Folder '" << folderPath << "' already exists." << endl;
        return folderPath;  //return;
    }
    // Create the folder
    if (createDirectory(folderPath)) { cout << "Folder '" << folderPath << "' created successfully." << endl; }
    else { cerr << "Failed to create folder '" << folderPath << "'." << endl; }
    return folderPath;      //return;
}

////////////////////////////// PRINTS //////////////////////////////

string getCurrentTime() {
    auto now = chrono::system_clock::now();
    time_t time = chrono::system_clock::to_time_t(now);
    tm localTime;                       // Convert time to local time
    localtime_s(&localTime, &time);
    char output[80];
    //string output;
    strftime(output, sizeof(output), "%y-%m-%d_%H-%M-%S", &localTime);
    return string(output);
}

string addTimeToFilename(string file_name) {
	auto now = chrono::system_clock::now();
	time_t time = chrono::system_clock::to_time_t(now);
	tm localTime;                       // Convert time to local time
	localtime_s(&localTime, &time);
	//stringstream ss;                    // Create a string stream to format the date and time
	char buffer[80];
	strftime(buffer, sizeof(buffer), "%y-%m-%d_%H-%M-%S ", &localTime);
	file_name = buffer + file_name;     // file_name = "drone_routes/" + string(buffer) + file_name;
	return file_name;
}

void csvPrintStops(const string& file_name) {
    ofstream outputFile("outputs/" + inst.time + "/" + file_name + ".csv");    // create .csv file from string name
    if (outputFile.is_open()) {
        outputFile << "ClusterID" << "," << "PtID" << "," << "X" << "," << "Y" << "\n";
        for (const auto& reef : inst.reefs) {
			outputFile << "," << reef.ID << "," << reef.x << "," << reef.y << "\n";
		}
        outputFile.close();
        cout << "\nReef stop points saved to: " << file_name << ".csv\n";
    }
    else { cerr << "Failed to open the output file.\n"; }
}

/// <summary>
/// 
/// </summary>
/// <param name="clusters"></param>
/// <param name="file_name"></param>
/// <param name="kMeansIters"></param>
void csvPrintClusters(const vector<ClusterSoln*>& clusters, string file_name="clusters", const string& in_folder = "") {
    ofstream outfile(in_folder + "/" + file_name + ".csv");

    if (!outfile.is_open()) {
        cerr << "Error: Unable to open clusters.csv for writing\n";
        return;
    }
    outfile << "ReefID,X,Y,ClusterID,,kMeansIters," << inst.kMeansIters << ",,W_MS," << inst.weights.first << ",,W_D," << inst.weights.second << "\n";    // Write header

    // Helper lambda function to write each reef's attributes
    auto writeReef = [&](const Pt* reef, int clusterID) {
        outfile << reef->ID << "," << reef->x << "," << reef->y << "," << clusterID << "\n";
        };

    // Iterate over clusters and write reefs' attributes
    for (const auto& cluster : clusters) {
        const auto& reefs = cluster->reefs;
        int clusterID = cluster->ID;
        for_each(reefs.begin(), reefs.end(), [&](const Pt* reef) {
            writeReef(reef, clusterID);
            });
    }
    outfile.close();
    cout << "Clusters saved to: " << file_name << ".csv\n";

}

/// <summary>
/// Print the launchPts to a CSV file
/// </summary>
/// <param name="routes"></param>
/// <param name="file_name">=true</param>
/// <param name="in_out"></param>
void csvPrintLaunchPts(vector<Pt*> launch_route, string file_name, const string& in_folder = "", bool print=true) {
    ofstream outputFile(in_folder + "/" + file_name + ".csv");

    if (outputFile.is_open()) {
		outputFile << "ID,X,Y\n";                                 // skip header row
		for (const auto& stop : launch_route) {                // for each stop in route
			outputFile << stop->ID << "," << stop->x << "," << stop->y << "\n";                  // output each stop across each row
		}
		outputFile.close();
        if (print) cout << "MS_launchpt coordinates saved to: " << file_name << ".csv\n";
	}
	else { cerr << "!! csvPrintLaunchPts: Failed to open the output file.\n"; }
	return;
}

/// <summary>
/// 
/// </summary>
/// <param name="launch_route"></param>
/// <param name="file_name"></param>
/// <param name="msDist">=NULL</param>
/// <param name="print">=true</param>
/// <param name="folder_name"></param>
void csvPrintMSRoutes(vector<Pt*> launch_route, string file_name, double msDist, const string& in_folder, bool print=true) {
    ofstream outputFile(in_folder + "/" + file_name + ".csv");

    Pt depot = inst.getDepot();
    if (outputFile.is_open()) {
        outputFile << "X,Y,msDist," << msDist << "\n";                                 // skip header row
        outputFile << depot.x << "," << depot.y << "\n";                  // output depot
        for (const auto& stop : launch_route) {                // for each stop in route
                outputFile << stop->x << "," << stop->y << "\n";                  // output each stop across each row
        }
        outputFile << depot.x << "," << depot.y << "\n";                  // output depot
        //outputFile << "\n";                             // go to next row after all stops in route have been output
        outputFile.close();
        if (print) cout << "MS route coordinates saved to: " << file_name << ".csv\n";
    }
    else { cerr << "!! csvPrintMSRoutes: Failed to open the output file.\n"; }
    return;
}

/// <summary>
/// 
/// </summary>
/// <param name="routes"></param>
/// <param name="file_name"></param>
/// <param name="in_out">=NULL</param>
/// <param name="print">=true</param>
/// <param name="folder_name"></param>
void csvPrintDroneRoutes(vector<vector<Pt*>> routes, string file_name, const string& in_folder, bool print=false) {
    ofstream outputFile(in_folder + "/" + file_name + ".csv");

    if (outputFile.is_open()) {
        outputFile << "\n";                                 // skip header row
        for (const auto& route : routes) {                  // for each route in routes
            for (const auto& stop : route) {                // for each stop in route
                outputFile << stop->ID << ","/* << stop->x << "," << stop->y << ","*/;                  // output each stop across each row
            }
            outputFile << "\n";                             // go to next row after all stops in route have been output
        }
        outputFile.close();
        if (print) cout << "Drone route points saved to: " << file_name << ".csv\n";
    }
    else { cerr << "!! csvPrintDroneRoutes: Failed to open the output file.\n"; }
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

//string csvPrintSA(vector<double> solution_best_dist, vector<double> solution_current_dist, vector<double> solution_new_dist, 
//    vector<double> solution_temp, int initial_temperature, double cooling_rate, 
//    int num_iterations, string file_name, string c = NULL) {
//    auto now = chrono::system_clock::now();
//    time_t time = chrono::system_clock::to_time_t(now);
//    tm localTime;                                   // Convert time to local time
//
//    localtime_s(&localTime, &time);
//    stringstream ss;                                    // Create a string stream to format the date and time
//    char buffer[80];
//    strftime(buffer, sizeof(buffer), "%y-%m-%d_%H-%M-%S ", &localTime);
//    file_name = buffer + file_name + '-' + c;
//    file_name = "sa_plots/" + file_name;
//    cout << "\n" << string(30, '*') << "\nSaving points to: " << file_name << ".csv\n" << string(30, '.') << "\n";
//
//    ofstream outputFile(file_name + ".csv");   // create .csv file from string name
//    outputFile << "Best_dist,Current_dist,New_dist,Temp,,initial_temperature," << initial_temperature
//        << ",,cooling_rate," << cooling_rate << ",,best_soln," << solution_best_dist.back()
//        << ",,iter," << num_iterations << "\n";                                 // skip header row
//    if (outputFile.is_open()) {
//        for (int i = 0; i < solution_best_dist.size() - 1;i++) {
//            outputFile << solution_best_dist[i] << ","          // output each solution
//                << solution_current_dist[i] << ","
//                << solution_new_dist[i] << ","
//                << solution_temp[i] << "\n";
//        }
//        outputFile << "\n";                             // go to next row after all stops in route have been output
//        outputFile.close();
//        cout << "SA Points saved to: " << file_name << ".csv\n" << string(30, '*') << "\n";
//    }
//    else { cerr << "!! PRINT SA SOLNS: Failed to open the output file.\n"; }
//    return file_name;
//}

//void csvPrintSA_Time(string file_name, chrono::seconds elapsed_time) {
//    int minutes = elapsed_time.count() / 60;
//    int seconds = elapsed_time.count() % 60;
//    //file_name = "sa_plots/" + file_name;
//    ofstream outputFile(file_name + ".csv", ios::app);   // create .csv file from string name
//    if (outputFile.is_open()) {
//		outputFile << ",,,,Time," << minutes << ",m, " << seconds << ",s\n";                                 // skip header row
//		outputFile.close();
//		cout << "Time saved to: " << file_name << ".csv\n" << string(30, '*') << "\n";
//	} else { cerr << "!! PRINT SA SOLNS: Failed to open the output file.\n"; }
//    return;
//}

string csvPrintSA(SAlog log, string file_name, const string& in_folder = "") {
    
    //size_t pos = in_folder.rfind('/', in_folder.rfind('/') - 1);  // Find the position of the second-to-last '/'
    size_t last_slash_pos = in_folder.find_last_of('/');     // Find the position of the last '/'

    string new_path;
    if (last_slash_pos != string::npos) {
        new_path = in_folder.substr(0, last_slash_pos);         // Extract the substring up to the position of the second-to-last '/'
        cout << "Modified path: " << new_path << endl;
    }
    else { cerr << "Invalid path format." << std::endl; }
    ofstream outputFile(new_path + "/" + file_name + ".csv");

    if (outputFile.is_open()) {
        outputFile << "temp,current_dist,new_dist,best_dist,,cooling rate," << log.params.cooling_rate << "\n";
        for (int i = 0; i < log.temp.size(); i++) { outputFile << log.temp[i] << "," << log.current_dist[i] << "," << log.new_dist[i] << "," << log.best_dist[i] << "\n"; }
        outputFile.close();
    }
    else printf("Unable to open file");
    return file_name;
}

/// <summary>
/// csv output of the full solution - INIT and FINAL
/// </summary>
/// <param name="best_new"></param>
/// <param name="file_suffix"></param>
/// <param name="path"></param>
void csvPrints(FullSoln best_new, string file_suffix, int run_iteration = NULL) { // csv output of the full solution - INIT and FINAL
    string mod_time = getCurrentTime() + "_Full_" + file_suffix;
    string folderPath = createFolder(to_string(run_iteration), mod_time);

    //csvPrintStops(/*best_new.msSoln.clusters, */"reef_set");
    vector<vector<Pt*>> total_routes;// = in_out ? best_new.droneSolns : best_new.droneSolns_out;
    for (const auto& vehicle : best_new.droneSolns) {
        for (const auto& route : vehicle->routes) { total_routes.push_back(route); }
	}
    csvPrintClusters(best_new.msSoln.clusters, "clusters", folderPath);
    csvPrintLaunchPts(best_new.msSoln.launchPts, "launchPts", folderPath);//"launchPts_fullSoln_" + boolToString(in_out));
    csvPrintMSRoutes(best_new.msSoln.launchPts, "ms_route", best_new.msSoln.getDist(), folderPath);//_"+boolToString(in_out));
    csvPrintDroneRoutes(total_routes, "drone_routes", folderPath, true);
    if (best_new.sa_log.best_dist.size()>1) csvPrintSA(best_new.sa_log, "sa_log", folderPath);
    //csvPrintSA_Time(filename_SA, fn_elapsed_time);
    return;
}

/// <summary>
/// csv output of the full solution - on the fly every swap update
/// </summary>
/// <param name="best_new"></param>
/// <param name="in_out"></param>
/// <param name="numUpdate"></param>
/// <param name="folder"></param>
void csvUpdate(FullSoln best_new, bool in_out, int numUpdate, int run_iteration=NULL) {     // csv output of the full solution - on the fly every swap update
    //string mod_time = getCurrentTime();// +"_OUT";
    string mod_time = to_string(numUpdate);
    if (in_out) { mod_time += "_IN"; }
    else { mod_time += "_OUT"; }
    string folderPath = createFolder(to_string(run_iteration), mod_time);
    vector<vector<Pt*>> total_routes;// = in_out ? best_new.droneSolns : best_new.droneSolns_out;
    for (const auto& vehicle : best_new.droneSolns) {
        for (const auto& route : vehicle->routes) { total_routes.push_back(route); }
    }
    csvPrintClusters(best_new.msSoln.clusters, "clusters", folderPath);
    csvPrintLaunchPts(best_new.msSoln.launchPts, "launchPts", folderPath, false);//"launchPts_fullSoln_" + boolToString(in_out));
    csvPrintMSRoutes(best_new.msSoln.launchPts, "ms_route", best_new.msSoln.getDist(), folderPath, false);//_"+boolToString(in_out));
    csvPrintDroneRoutes(total_routes, "drone_routes", folderPath);
    return;
}
//void csvUpdate_IN(FullSoln best_new) {
//    string mod_time = getCurrentTime() + "_IN";
//    createFolder(mod_time);
//    vector<vector<Pt*>> total_routes;// = in_out ? best_new.droneSolns : best_new.droneSolns_out;
//    for (const auto& vehicle : best_new.droneSolns) {
//        for (const auto& route : vehicle->routes) { total_routes.push_back(route); }
//    }
//    csvPrintDroneRoutes(total_routes, "drone_routes", mod_time);
//    return;
//}
//void csvUpdate_OUT(FullSoln best_new) {
//    string mod_time = getCurrentTime() + "_OUT";
//    createFolder(mod_time);
//    vector<vector<Pt*>> total_routes;// = in_out ? best_new.droneSolns : best_new.droneSolns_out;
//    for (const auto& vehicle : best_new.droneSolns) {
//        for (const auto& route : vehicle->routes) { total_routes.push_back(route); }
//    }
//    csvPrintLaunchPts(best_new.msSoln.launchPts, "launchPts", mod_time, false);//"launchPts_fullSoln_" + boolToString(in_out));
//    csvPrintMSRoutes(best_new.msSoln.launchPts, "ms_route", best_new.msSoln.getDist(), mod_time, false);//_"+boolToString(in_out));
//    csvPrintDroneRoutes(total_routes, "drone_routes", mod_time);
//    return;
//}

///// <summary>
///// 
///// </summary>
///// <param name="launchPts"></param>
///// <param name="msDist"></param>
///// <param name="method"></param>
//void csvPrintMS(const vector <Pt*> launchPts, double msDist, string method) {
//    string mod_time = getCurrentTime() + "_MS_init";
//    createFolder(mod_time);
//
//    csvPrintMSRoutes(launchPts, "ms_route_"+method, msDist, mod_time);
//    csvPrintLaunchPts(launchPts, "launchPts_"+method, mod_time);//"launchPts_fullSoln_" + boolToString(in_out));
//
//    return;
//}