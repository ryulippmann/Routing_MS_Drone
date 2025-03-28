#pragma once
#include <sstream>
#include <fstream>
#include <filesystem>
#include <unordered_set>

#ifdef _WIN32
#include <direct.h>   // For mkdir on Windows
//#else
//#include <unistd.h>
#endif

bool directoryExists(const string& path) {
    struct stat info;
    return stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

void writeOpt_RuntimeToFile(const string& filename, int iter, double best_dist, const chrono::duration<double>& elapsed) {
    int minutes = static_cast<int>(elapsed.count()) / 60;
    int seconds = round(fmod(elapsed.count(), 60));

    ofstream outfile(filename + ".csv", ios::app); // Open the file in append mode
    if (outfile.is_open()) {
		if (iter == 0) { // If first iteration, write header
			outfile << "Iteration,WEIGHTED Total dist,,Mins,Secs\n";
		}
        outfile << iter << "," << best_dist << ",," << minutes << "," << seconds << "\n"; // Add runtime information
        outfile.close();
    }
    else {
        cerr << "Unable to open file " << filename << ".csv" << endl;
    }
}

bool createDirectory(const string& path) {
    stringstream ss(path);
    string item;
    string currentPath = "";

    while (getline(ss, item, '/')) {
        if (!item.empty()) {  // Avoid empty tokens
            currentPath += item + "/";
            if (!directoryExists(currentPath)) {
#ifdef _WIN32
                if (_mkdir(currentPath.c_str()) != 0) {
#else
                if (mkdir(currentPath.c_str(), 0777) != 0) { // Note: 0777 sets full access permissions
#endif
                    cerr << "Failed to create folder '" << currentPath << "'." << endl;
                    return false;
                }
            }
        }
    }
    return true;
}

string createFolder(const string& name, const string& folder_name = "", const string& sub_folder ="") {
    string folder_path;
    // if name doesn't starts with "outputs/", add it
    if (name.substr(0, 8) != "outputs/") {
        folder_path = "outputs/" + name;    // Define the path of the folder
	}
    else {
		folder_path = name;
	}

    if (!folder_name.empty()) folder_path += "/" + folder_name;
    if (!sub_folder.empty()) folder_path += "/" + sub_folder;

    if (directoryExists(folder_path)) {    // Check if the folder already exists
        if (print_general) cout << "Folder '" << folder_path << "' already exists." << endl;
        return folder_path;
    }
    // Create the folder
    if (!createDirectory(folder_path)) //{ cout << "Folder '" << folder_path << "' created successfully." << endl; }
    /*else*/ { cerr << "Failed to create folder '" << folder_path << "'." << endl; }
    return folder_path;
}

void createRunFolder(const string& folder_name, string run_name) {
    string folder_path = "outputs/" + folder_name + "/" + run_name;    // Define the path of the folder
    if (directoryExists(folder_path)) {    // Check if the folder already exists
        if (print_general) cout << "Folder '" << folder_path << "' already exists." << endl;
        return;
    }
    // Create the folder
    if (createDirectory(folder_path)) { cout << "Folder '" << folder_path << "' created successfully." << endl; }
    else { cerr << "Failed to create folder '" << folder_path << "'." << endl; }
    return;
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

void csvPrintStops(const Problem& inst, const string& folder_name, const string& file_name) {
    ofstream outputFile(folder_name + "/" + file_name + ".csv");    // create .csv file from string name
    if (outputFile.is_open()) {
        outputFile << "ClusterID" << "," << "PtID" << "," << "X" << "," << "Y" << "\n";
        for (const auto& reef : inst.reefs) {
			outputFile << "," << reef.ID << "," << reef.x << "," << reef.y << "\n";
		}
        outputFile.close();
        if (print_general) cout << "\nReef stop points saved to: " << file_name << ".csv\n";
    }
    else { cerr << "Failed to open the output file.\n"; }
}

/// <summary>
/// print drone routes and dists - based on droneSoln* drone
/// Only called in IN_ClusterSwaps
/// </summary>
/// <param name="drone"></param>
void printDroneRoutes(const DroneSoln* drone) {
    for (const auto& route : drone->routes) {
        for (auto& node : route) {
            printf("\t%d ->", node->ID);
        }
        printf("\nDrone route dist:\t%f\n", drone->getDroneRouteDist(route));
    }
}

/// <summary>
/// 
/// </summary>
/// <param name="clusters"></param>
/// <param name="file_name"></param>
/// <param name="kMeansIters"></param>
void csvPrintClusters(const vector<ClusterSoln*>& clusters, const Problem& inst, string file_name="clusters", const string& in_folder = "", int no_iters=NULL) {
    ofstream outfile(in_folder + "/" + file_name + ".csv");

    if (!outfile.is_open()) {
        cerr << "Error: Unable to open clusters.csv for writing\n";
        return;
    }
    outfile << "ReefID,X,Y,ClusterID,,kMeansIters," << inst.kMeansIters << ",,W_MS," << inst.weights.first << ",,W_D," << inst.weights.second << ",Total_iters," << no_iters << "\n";    // Write header

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
    if (print_general) cout << "Clusters saved to: " << file_name << ".csv\n";
}

/// <summary>
/// Print the launchPts to a CSV file
/// </summary>
void csvPrintLaunchPts(const vector<Pt*>& launch_route, string file_name, const string& in_folder = "") {
    ofstream outputFile(in_folder + "/" + file_name + ".csv");

    if (outputFile.is_open()) {
		outputFile << "ID,X,Y\n";                                 // skip header row
		for (const auto& stop : launch_route) {                // for each stop in route
			outputFile << stop->ID << "," << stop->x << "," << stop->y << "\n";                  // output each stop across each row
		}
		outputFile.close();
        if (print_general) cout << "MS_launchpt coordinates saved to: " << file_name << ".csv\n";
	}
	else { cerr << "!! csvPrintLaunchPts: Failed to open the output file.\n"; }
	return;
}

void csvPrintMSRoutes(const vector<Pt*>& launch_route, const Pt& depot, string file_name, double msDist, const string& in_folder) {
    ofstream outputFile(in_folder + "/" + file_name + ".csv");
    if (outputFile.is_open()) {
        outputFile << "X,Y,msDist," << msDist << "\n";                                 // skip header row
        outputFile << double(depot.x) << "," << double(depot.y) << "\n";                  // output depot
        for (const auto& stop : launch_route) {                // for each stop in route
                outputFile << stop->x << "," << stop->y << "\n";                  // output each stop across each row
        }
        outputFile << depot.x << "," << depot.y << "\n";                  // output depot
        //outputFile << "\n";                             // go to next row after all stops in route have been output
        outputFile.close();
        if (print_general) cout << "MS route coordinates saved to: " << file_name << ".csv\n";
    }
    else { cerr << "!! csvPrintMSRoutes: Failed to open the output file.\n"; }
    return;
}

void csvPrintDroneRoutes(const vector<vector<Pt*>>& routes, string file_name, const string& in_folder, bool print=false) {
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
        if (print_general) cout << "Drone route points saved to: " << file_name << ".csv\n";
    }
    else { cerr << "!! csvPrintDroneRoutes: Failed to open the output file.\n"; }
    return;
}

string csvPrintSA(SAlog log, string file_name, const string& in_folder) {
    ofstream outputFile(in_folder + "/" + file_name + ".csv");

    if (outputFile.is_open()) {
        outputFile << "iter8,temp,current_dist,new_dist,best_dist,,cooling rate," << log.params.cooling_rate << "\n";
        for (int i = 0; i < log.temp.size(); i++) { outputFile << i << "," << log.temp[i] << "," << log.current_dist[i] << "," << log.new_dist[i] << "," << log.best_dist[i] << "\n"; }
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
void csvPrints(const FullSoln& best_new, const Problem& inst, string file_suffix, int run_iteration, int no_iters, string batch = NULL) { // csv output of the full solution - INIT and FINAL
    string mod_time = getCurrentTime() + "_Full_" + file_suffix;
    string folder_path;
    if (!batch.empty()) {
        folder_path = createFolder(batch, to_string(run_iteration), mod_time);
    }
    else {
        folder_path = createFolder(inst.time, to_string(run_iteration), mod_time);
    }

    vector<vector<Pt*>> total_routes;
    for (const auto& vehicle : best_new.droneSolns) {
        for (const auto& route : vehicle->routes) { total_routes.push_back(route); }
	}

    vector<Pt*> launchPts = best_new.msSoln.launchPts;
    unordered_set<int> uniqueIDs;
    for (const auto& point : launchPts) { uniqueIDs.insert(point->ID); }

    for (const auto& drone : best_new.droneSolns) {
        if (uniqueIDs.find(drone->launchPts.first->ID) == uniqueIDs.end()) {
            launchPts.push_back(drone->launchPts.first);
            uniqueIDs.insert(drone->launchPts.first->ID);
        }
        if (uniqueIDs.find(drone->launchPts.second->ID) == uniqueIDs.end()) {
            launchPts.push_back(drone->launchPts.second);
            uniqueIDs.insert(drone->launchPts.second->ID);
        }
    }

    csvPrintClusters(best_new.msSoln.clusters, inst, "clusters", folder_path, no_iters);
    csvPrintLaunchPts(/*best_new.msSoln.*/launchPts, "launchPts", folder_path);
    csvPrintMSRoutes(best_new.msSoln.launchPts, best_new.msSoln.ms.depot, "ms_route", best_new.msSoln.getDist(), folder_path);
    csvPrintDroneRoutes(total_routes, "drone_routes", folder_path, true);
    if (best_new.sa_log.best_dist.size()>1) csvPrintSA(best_new.sa_log, "sa_log", batch + "/" + to_string(run_iteration));
    //csvPrintSA_Time(filename_SA, fn_elapsed_time);
    return;
}

/// <summary>
/// csv output of the full solution - on the fly every swap update
/// </summary>
/// <param name="best_new"></param>
/// <param name="out_in_sortie"></param>
/// <param name="numUpdate"></param>
/// <param name="folder"></param>
void csvUpdate(const FullSoln& best_new, const Problem& inst, int out_in_sortie, int numUpdate, int no_iters=NULL, string folder_path = "") {     // csv output of the full solution - on the fly every swap update
    //string mod_time = getCurrentTime();// +"_OUT";
    string mod_time = to_string(numUpdate);
    if (out_in_sortie == 0) { mod_time += "_OUT"; }
    else if (out_in_sortie == 1) { mod_time += "_IN"; }
    else if (out_in_sortie == 2) { mod_time += "_SORTIE"; }
	else { mod_time += "_ERROR"; }
    folder_path = createFolder(folder_path, mod_time);
    vector<vector<Pt*>> total_routes;// = out_in_sortie ? best_new.droneSolns : best_new.droneSolns_out;
    for (const auto& vehicle : best_new.droneSolns) {
        for (const auto& route : vehicle->routes) { total_routes.push_back(route); }
    }
    csvPrintClusters(best_new.msSoln.clusters, inst, "clusters", folder_path, no_iters);
    csvPrintLaunchPts(best_new.msSoln.launchPts, "launchPts", folder_path);//"launchPts_fullSoln_" + boolToString(out_in_sortie));
    csvPrintMSRoutes(best_new.msSoln.launchPts, best_new.msSoln.ms.depot, "ms_route", best_new.msSoln.getDist(), folder_path);//_"+boolToString(out_in_sortie));
    csvPrintDroneRoutes(total_routes, "drone_routes", folder_path);
    return;
}

void printClusters(const vector<ClusterSoln*>& clusters) {
    for (int i = 0; i < clusters.size(); i++) {
        printf("\tCluster: %d\n", i);					// Print clusters
        for (int j = 0; j < clusters[i]->reefs.size(); j++) {	// for reefs in cluster
            printf("%d\t(%.2f, %.2f)\n", clusters[i]->reefs[j]->ID, clusters[i]->reefs[j]->x, clusters[i]->reefs[j]->y);
        } // print ID (x,y) for each reef in cluster
        printf("Centroid:\t\t%d\t(%.2f, %.2f)\n", clusters[i]->getCentroid().ID, clusters[i]->getCentroid().x, clusters[i]->getCentroid().y);
    } // for each cluster

    return;
}

void printOpts(const Problem& inst, const vector<FullSoln>& fullSolns, string sens_run) {
    string folder_name = createFolder(inst.time + sens_run);
    ofstream outputFile(folder_name + "/opts.csv");
    if (outputFile.is_open()) {
        outputFile << "Run,MS dist,Drone dist,WEIGHTED Total dist\n";
        for (int i = 0; i < fullSolns.size(); i++) {
            double msDist = fullSolns[i].msSoln.getDist();

            double droneDist = 0;
            for (int d = 0; d < fullSolns[i].droneSolns.size(); d++) {
                droneDist += fullSolns[i].droneSolns[d]->getDroneDist();
            }

            double totalDist = fullSolns[i].getTotalDist(inst.weights);// msDist + droneDist;

            outputFile << i << "," << msDist << "," << droneDist << "," << totalDist << "\n";
        }
        outputFile.close();
        cout << "Optimal solutions saved to: " << folder_name << "/opts.csv\n";
    }
    else { cerr << "Failed to open the output file.\n"; }
}
