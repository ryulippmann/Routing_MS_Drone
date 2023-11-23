#pragma once


//vector<vector<double>> calc_centMatrix(pair<double, double> depot, vector <ClusterPoint> centroids, bool print = false) {
//    if (centroids[0].x != depot.first && centroids[0].y != depot.second) {
//        centroids.insert(centroids.begin(), ClusterPoint(depot.first, depot.second));
//    }
//    vector<vector<double>> centMatrix(centroids.size(), vector<double>(centroids.size(), 0));
//    //vector<double> depotDist;
//    //depotDist.push_back(0);
//    //for (const ClusterPoint& c : centroids) {
//    //    depotDist.push_back(sqrt(pow((depot.first - c.x),2) + pow((depot.second - c.y),2)));
//    //}
//    for (int i = 0; i < centroids.size(); i++) {
//        for (int j = 0; j < centroids.size(); j++) {
//            centMatrix[i][j] = (sqrt(pow((centroids[i].x - centroids[j].x), 2) + pow(centroids[i].y - centroids[j].y, 2)));
//        }
//    }
//    //centMatrix.insert(centMatrix.begin(), depotDist);
//    //for (int a = 1; a < centMatrix.size(); a++) { centMatrix[a].insert(centMatrix[a].begin(), depotDist[a]); }
//
//    if (print) {
//        printf("\nFrom depot to cluster centroids:\n");     // print centroidMatrix
//        for (int i = 0; i < centroids.size(); i++) printf("\t%d", i);
//        printf("\n");
//        int j = 0;
//        for (const vector<double>& line : centMatrix) { printf("%d", j); for (const double& dist : line) { printf("\t%2.2f", dist); } cout << "\n"; j++; }
//    }
//    return centMatrix;
//}

pair<vector<pair<double, double>>, double> setLaunchPts(const vector<int> clusterOrder, vector<ClusterPoint> centroids, const pair<double, double> depot, bool print = false) {
    vector<pair<double, double>> launchpts;
    launchpts.push_back(depot);
    centroids.insert(centroids.begin(), ClusterPoint(depot.first, depot.second));
    // using clusterOrder,
    for (int c = 0; c < clusterOrder.size() - 1; c++) {
        pair<double, double> launchpt = make_pair( // sum adjacent clusters x,y's to calc launchpts and 
            (centroids[clusterOrder[c]].x + centroids[clusterOrder[c + 1]].x) / 2,
            (centroids[clusterOrder[c]].y + centroids[clusterOrder[c + 1]].y) / 2);
        if (print) printf("\n%d -> %d\t(%2.2f, %2.2f)", clusterOrder[c], clusterOrder[c + 1], launchpt.first, launchpt.second);
        launchpts.push_back(launchpt); // store these coords in launchpts
    }
    //pair<double, double> launchpt = make_pair( // sum adjacent clusters x,y's to calc launchpts and 
    //    (centroids[clusterOrder[clusterOrder.size() - 1]].x + centroids[clusterOrder[0]].x) / 2,
    //    (centroids[clusterOrder[clusterOrder.size() - 1]].y + centroids[clusterOrder[0]].y) / 2);
    //if (print) printf("\nclusters here are one index too large.\nHere depot = 0");
    ////printf("\n%d -> %d\t(%2.2f, %2.2f)", clusterOrder[clusterOrder.size() - 1], clusterOrder[0], launchpt.first, launchpt.second);
    //launchpts.push_back(launchpt); // store these coords in launchpts

    //launchpts.push_back(depot);       // Add this line if Greedy function NOT run
    double route_dist = 0;
    for (int s = 0; s < launchpts.size() - 1; s++) {
        route_dist += calcPtDist(launchpts[s], launchpts[s + 1]);
    }
    pair<vector<pair<double, double>>, double> ms_launchpt_route = make_pair(launchpts, route_dist);
    return ms_launchpt_route;
}
