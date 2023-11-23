#pragma once
#include <vector>
#include <cmath>
using namespace std;
#include "class_def.h"

// RETURN reefs visited
int reefsVisited(const vector<Pt*>& reefs) { 
    int visit_count = 0;
    vector<bool> visited(reefs.size(), false);
    for (int i = 0; i < reefs.size(); i++) {
        if (visited[i] == true) { visit_count++; }
    }
    return visit_count;
}//reefsVisited



//vehicles argument not referenced, so are not updated...
vector<vector<Pt*>> droneWithinClusterNearestNeighbour(const MSSoln* ms, const int c,
    bool csv_print = false) {
    const Problem* inst = ms->inst;
    const Cluster* cluster = ms->clustSoln->clusters[c];
    pair<Pt*, Pt*> launchPts = make_pair(ms->launchPts[c], ms->launchPts[c + 1]);
    //int reefs_visit_count = reefsVisited(cluster->reefs);    // int of number of reefs visited
    int u = -1;                                          // initialise current reef index
    vector <double> route_dists;//(vehicles.size(),0.0); // list of route dist for each vehicle
    //vector<vector<int>> route_stops;                     // create routes to output to csv
    bool printStops = false;
    if (printStops) { cout << "\n\nNEAREST NEIGHBOUR\n"; }
    vector<vector<Pt*>> routes(inst->tenders.size(), vector<Pt*>(inst->tenders[c].cap, nullptr)); // create routes to save in TenderSoln.routes
    vector<bool> visited(cluster->reefs.size(), false);
    const vector<vector<double>> dMatrix = ms->clustSoln->clusters[c]->getdMatrix(c, make_pair(ms->launchPts[c], ms->launchPts[c+1]));//[u];                        // for u vector in dMatrix

    for (int m = 0; m < inst->tenders.size() /*&& reefs_visit_count < cluster->reefs.size() - 2*/; m++) {     //FOR EACH VEHICLE!!
        vector<Pt*> row;                            // create row for routes
        // add drop off pt to row
        double route_dist = 0;
        if (row/*cluster.drones[m].reef_stops*/.size() == 0) {                       // if new vehicle = no reefs assigned to vehicle
            row.push_back(launchPts.first);
            u = 0;
            //u = rand() % reefs.size();                                  // assign random unvisited reef     // Select an arbitrary vertex, set it as the current vertex u. 
            //while (reefs[u].visited != -1) {                          // while current reef has already been visited
            //    u = rand() % reefs.size();                              // assign current reef to random unvisited reef
            //}//while
            //reefs[u].visited = m;                                    // Mark u as visited.
            //reefs_visit_count = reefsVisited(cluster->reefs);                    // int of number of reefs visited
            //cluster.drones[m].reef_stops
            //row.push_back(cluster->reefs[u]);                        // append current reef to current vehicle reef stop list
        }//if(v.stops==0)       vv      // 3: Find the shortest edge connecting current vertex u and an unvisited vertex v.
        for (int k = 0; k < inst->tenders[m].cap/* && reefs_visit_count < cluster->reefs.size()*/; k++) {            // for stops within vehicle capacity
            //                          msSoln.clustSoln->clusters[c]->getdMatrix
            
            const vector<double>& neighbours = dMatrix[u];//cluster->getdMatrix(m, make_pair(ms->launchPts[m], ms->launchPts[m]));//[u];                        // for u vector in dMatrix
            double min = DBL_MAX;                                       // initialise min dist as MAX
            int v = -1;                                                 // initialise index as -1

            for (int w = 1; w < cluster->reefs.size()+2; w++) {
                if (neighbours[w] < min && visited[w-1] == false /*&& cluster->reefs[w].id != -1 && cluster->reefs[w].id != -2*/) { // if closer unvisited reef:
                    v = w;                                              // update index
                    min = neighbours[v];                                // update min
                }//if
            }//for
            /*cluster.drones[m].*/route_dist += min;                              // add dist travelled to route dist
            u = v;                                                      // Update u=v as new index of closest pt
            /*cluster->reefs[u].*/visited[u-1] = true;                                    // Mark u=v(new) as visited.
            row.push_back(cluster->reefs[u-1]);
            //reefs_visit_count = reefsVisited(cluster->reefs);
            //if (reefs[u].visited != -1) {cout << reefs_visit_count << " reefs visited  \tCurrent reef: " << u << "  \tUpdated route_dist = \t" << vehicles[m].route_dist << endl;}//if
        }//for(v.capacity)
        // when one stop left, go to pick up pt
        int v = cluster->reefs.size() + 1;
        route_dist += dMatrix[u][v];
        //reefs[v].visited = m;                                    // Mark u=v(new) as visited.
        row.push_back(launchPts.second);

        if (printStops) { cout << "Vehicle: " << m /*<< "\tcapacity: " << vehicles[m].capacity << endl*/ << "\tStops:\t"; //}    // Iterate through vehicle count
        //for (int q = 0; q < cluster.drones[m].reef_stops.size(); q++) {       // for each ordered reef stop
        //    if (printStops) { printf("%d\t", cluster.drones[m].reef_stops[q]); }                 // print each stop ID
        //    row.push_back(cluster.drones[m].reef_stops[q]);                   // append ordered stop ID to row
        //}//for(v.stops)
        /*if (printStops) {*/ printf("\n\tDist:    \t%f\t", route_dist); //}
        //Close the route loop. Return to initial node.
        /*if (printStops) {*/ cout << "Return from final node: " << row.back()->ID << "\t(" << row.back()->x << ", " << row.back()->y << ")\tback to node: "; 
        }
        row.push_back(row[0]);                  //append first stop to end of "row"
        //row.push_back(row[0]);    //append first stop to reef stops attribute of vehicle
        if (printStops) { cout << row.back() << "\tdone!\n"; }

        // find dMatrix index of 2nd last and last stops, add distance to route_dist
        // this dist should be ZERO because it's from drop to pick nodes...
        route_dist += dMatrix.back()[0];
        //int a = -1;
        //for (int i = 0; i < cluster->reefs.size(); ++i) {
        //    if (cluster->reefs[i]->ID == cluster->reefs[cluster->reefs.size() - 2]->ID) {
        //        a = i;
        //        break; // Return the index as an integer
        //    } //break;
        //}
        //int b = -1;
        //for (int j = 0; j < cluster->reefs.size(); ++j) {
        //    if (cluster->reefs[j]->ID == cluster->reefs.back()->ID) {
        //        b = j;
        //        break; // Return the index as an integer
        //    } //break;
        //}
        //route_dist += dMatrix[a][b];//vehicles[m].reef_stops[vehicles[m].capacity - 1]][vehicles[m].reef_stops[vehicles[m].capacity]];

        routes[m] = row;
        route_dists.push_back(route_dist);                  // append distance of vehicle route
        if (printStops) { cout << "Closed loop dist:\t" << route_dist << "\n\n"; }
    }//for(vehicles)


    //double total_route_dists = sum(route_dists);                        // sum of vehicle route dists
    if (printStops) { cout << "\nTotal vehicle route dist: " << sum(route_dists) << "\n"; }
    //double route_dist = printRouteStats(cluster.drones, cluster.dMatrix.size() / cluster.drones.size(), "NEAREST  NEIGHBOUR");
    //pair<vector<vector<Pt*>>, double> routess = make_pair(routes, sum(route_dists)/*route_dist*/);
    //if (csv_print) { csvPrintStops(cluster->reefs, "points"); csvPrintRoutes(routes.first, "nn_route_list"); }
    return /*cluster*/routes;
}//nearestNeighbour
