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
	string folderPath = "outputs/" + INST.time;    // Define the path of the folder
	if (!in_folder.empty()) folderPath += "/" + in_folder;
	return folderPath;
}

string createFolder(const string& folder_name = "", const string& sub_folder ="") {
    string folderPath = "outputs/" + INST.time;    // Define the path of the folder
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

void createRunFolder(int iter) {
    string folderPath = "outputs/" + INST.time + "/" + to_string(iter);    // Define the path of the folder
    if (directoryExists(folderPath)) {    // Check if the folder already exists
        cout << "Folder '" << folderPath << "' already exists." << endl;
        return;
    }
    // Create the folder
    if (createDirectory(folderPath)) { cout << "Folder '" << folderPath << "' created successfully." << endl; }
    else { cerr << "Failed to create folder '" << folderPath << "'." << endl; }
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

void csvPrintStops(const string& file_name) {
    ofstream outputFile("outputs/" + INST.time + "/" + file_name + ".csv");    // create .csv file from string name
    if (outputFile.is_open()) {
        outputFile << "ClusterID" << "," << "PtID" << "," << "X" << "," << "Y" << "\n";
        for (const auto& reef : INST.reefs) {
			outputFile << "," << reef.ID << "," << reef.x << "," << reef.y << "\n";
		}
        outputFile.close();
        cout << "\nReef stop points saved to: " << file_name << ".csv\n";
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
void csvPrintClusters(const vector<ClusterSoln*>& clusters, string file_name="clusters", const string& in_folder = "") {
    ofstream outfile(in_folder + "/" + file_name + ".csv");

    if (!outfile.is_open()) {
        cerr << "Error: Unable to open clusters.csv for writing\n";
        return;
    }
    outfile << "ReefID,X,Y,ClusterID,,kMeansIters," << INST.kMeansIters << ",,W_MS," << INST.weights.first << ",,W_D," << INST.weights.second << "\n";    // Write header

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

void csvPrintMSRoutes(vector<Pt*> launch_route, string file_name, double msDist, const string& in_folder, bool print=true) {
    ofstream outputFile(in_folder + "/" + file_name + ".csv");

    Pt depot = INST.getDepot();
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

string csvPrintSA(SAlog log, string file_name, const string& in_folder = "") {
    size_t last_slash_pos = in_folder.find_last_of('/');     // Find the position of the last '/'
    string new_path;
    if (last_slash_pos != string::npos) {
        new_path = in_folder.substr(0, last_slash_pos);         // Extract the substring up to the position of the second-to-last '/'
    }
    else { cerr << "Invalid path format." << endl; }
    ofstream outputFile(new_path + "/" + file_name + ".csv");

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

void printClusters(vector<ClusterSoln*> clusters) {
    for (int i = 0; i < clusters.size(); i++) {
        printf("\tCluster: %d\n", i);					// Print clusters
        for (int j = 0; j < clusters[i]->reefs.size(); j++) {	// for reefs in cluster
            printf("%d\t(%.2f, %.2f)\n", clusters[i]->reefs[j]->ID, clusters[i]->reefs[j]->x, clusters[i]->reefs[j]->y);
        } // print ID (x,y) for each reef in cluster
        printf("Centroid:\t\t%d\t(%.2f, %.2f)\n", clusters[i]->getCentroid().ID, clusters[i]->getCentroid().x, clusters[i]->getCentroid().y);
    } // for each cluster

    return;
}
