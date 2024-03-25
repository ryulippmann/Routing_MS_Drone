import csv
import matplotlib.pyplot as plt
from collections import OrderedDict
import datetime
import os
import re

cluster_soln = True
cluster_path =      'clusters/'+    '24-03-22_13-57-55 clusters_init'               +'.csv'
ms_soln = True
ms_path =           'ms_route/'+    '24-03-22_14-00-48 ms_launch_route_fullSoln'    +'.csv'
launchpts_path =    'launchPts/'+   '24-03-22_14-00-47 launchPts_fullSoln'          +'.csv'
drone_soln = True
d_path =            'd_route/'+     '24-03-22_14-00-48 drone_routes'            +'.csv'

w_ms = 5
w_d = 1

print_plt = True
save_plt = True


def plot_clusters(csv_file, print_plt, save_plt):
    # Read CSV file and extract data
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        data = list(reader)

    kmeans_iters = int(reader._fieldnames[6])
    # Extract x, y, cluster ID from the data
    x = [float(row['X']) for row in data]
    y = [float(row['Y']) for row in data]
    cluster_ids = [int(row['ClusterID']) for row in data]
    node_id = [int(row['ReefID']) for row in data]

    # Initialize nodes list with empty lists
    nodes = [[] for _ in range(len(node_id))]    
    for n,(i,j,k) in enumerate(zip(x,y,cluster_ids)):
        nodes[node_id[n]-1] = [i,j,k]

    # Get unique cluster IDs and assign colors
    unique_cluster_ids = list(OrderedDict.fromkeys(cluster_ids))
    num_clusters = len(unique_cluster_ids)
    colors = plt.cm.get_cmap('tab10', num_clusters)

    # Plot points colored by cluster ID
    plt.figure(figsize=(8, 6))
    i=0
    for cluster_id, color in zip(unique_cluster_ids, colors.colors):
        cluster_x = [x[i] for i in range(len(x)) if cluster_ids[i] == cluster_id]
        cluster_y = [y[i] for i in range(len(y)) if cluster_ids[i] == cluster_id]
        plt.scatter(cluster_x, cluster_y, label=f'Cluster {cluster_id}', color=color)


        # Annotate with cluster sequential number in centroid
        centroid_x = sum(cluster_x) / len(cluster_x)
        centroid_y = sum(cluster_y) / len(cluster_y)
        plt.annotate(f'{i}', xy=(centroid_x, centroid_y), xycoords='data', ha='center', va='center', fontsize=24, color=color, fontweight='bold')
        i+=1
    # Annotate each point with node_id
    for n, (ni, xi, yi) in enumerate(zip(node_id, x, y)):
        plt.annotate(f'{ni}', xy=(xi, yi), xycoords='data', ha='left', va='top', fontsize=10)
    # Annotate with kMeansIters
    plt.annotate(f'kMeansIters: {kmeans_iters}', xy=(0.95, 0.05), xycoords='axes fraction', ha='right', va='top', fontsize=10, bbox=dict(facecolor='white', alpha=0.5))

    # plt.title('Reefs Clustered by Cluster ID')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    plt.legend(loc='upper right')    # Show the legend in top right corner
    plt.grid(True)

    # annotate plot with cluster path
    plt.annotate(f'ClusterPath = {cluster_path}', xy=(0.01, -0.05), xycoords='axes fraction', ha='left', va='top')##, fontsize=12, fontweight='bold')

    date_time = re.search(r'(\d{2}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', cluster_path)
    launchpts_path = f'launchPts/{date_time.group(1)} launchPts_init.csv'
    if not ms_soln: 
        if (max([sublist[-1] for sublist in nodes])>0):
            get_launchpts(launchpts_path)     # launchpts returns a dictionary of launch points by ID = (x, y)

    if (save_plt):        # Save the plot in folder "plots" with the current date as the filename
        if not os.path.exists("plots"): # Create the "plots" folder if it doesn't exist
            os.makedirs("plots")
            if not os.path.exists("plots/clustering"): # Create the "plots" folder if it doesn't exist
                os.makedirs("plots/clustering")
        current_datetime = datetime.datetime.today().strftime('%Y%m%d_%H-%M-%S')
        plt.savefig(f"plots/clustering/{current_datetime}__kM={kmeans_iters}.png")
    if (print_plt and not (ms_soln or drone_soln)) : plt.show()
    return nodes

def Dist(a,b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5

def get_launchpts(csv_file):
    # Read CSV file and extract data
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        data = list(reader)

    # Extract x, y coordinates from the data using list comprehension
    launchpts = {int(row['ID']): (float(row['X']), float(row['Y'])) for row in data}

    # Plot launch points on the existing plot
    x_values, y_values = zip(*launchpts.values())
    plt.scatter(x_values, y_values, color='black', marker='x', label='Launch Points')
    # annotate plot with cluster path
    # plt.annotate(f'LaunchPath = {cluster_path}', xy=(0.01, -0.1), xycoords='axes fraction', ha='left', va='top')##, fontsize=12, fontweight='bold')

    return launchpts

def plot_MS_route(csv_file, print_plt, save_plt, w_ms, color = 'k'):
    # Read CSV file and extract data
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        data = list(reader)

    dist = float(reader._fieldnames[3])
    # Extract x, y coordinates from the data
    x = [float(row['X']) for row in data]
    y = [float(row['Y']) for row in data]
    if (dist==0): 
        for i in range(len(x)-1): 
            dist += w_ms * Dist([x[i], y[i]], [x[i+1], y[i+1]])  # ((x[i]-x[i+1])**2 + (y[i]-y[i+1])**2)**0.5
        # dist = round(sum([((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2)**0.5 for i in range(1,len(x))]),2)
    
    # Annotate the plot with the coordinates of the first point as "depot"
    plt.annotate(f'Depot: ({x[0]}, {y[0]})', xy=(x[0], y[0]), xycoords='data', ha='left', va='top', fontsize=10, color='black')
    # if (csv_file.find('NN')):   plt.annotate(f'MS_NN = {dist:.2f}\n@w_ms = {w_ms}',     xy=(0.01, 1), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = color, fontweight='bold')
    # elif (csv_file.find('Gd')): plt.annotate(f'MS_Gd = {dist:.2f}\n@w_ms = {w_ms}',     xy=(0.01, 1), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = color, fontweight='bold')
    # else:                       
    plt.annotate(f'MS_dist = {dist:.2f}\n@w_ms = {w_ms}',   xy=(0.01, 1), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = color, fontweight='bold')
    # Plot MS route on the existing plot
    plt.plot(x, y, color, linestyle='-', linewidth=2)  # You can adjust color, linestyle, and linewidth as needed

    # annotate plot with cluster path
    plt.annotate(f'MS_Path = {ms_path}', xy=(0.01, -0.075), xycoords='axes fraction', ha='left', va='top')##, fontsize=12, fontweight='bold')

    if (save_plt):        # Save the plot in folder "plots" with the current date as the filename
        if not os.path.exists("plots"): # Create the "plots" folder if it doesn't exist
            os.makedirs("plots")
            if not os.path.exists("plots/ms_routing"): # Create the "plots" folder if it doesn't exist
                os.makedirs("plots/ms_routing")
        current_datetime = datetime.datetime.today().strftime('%Y%m%d_%H-%M-%S')
        plt.savefig(f"plots/ms_routing/{current_datetime}.png")
    if (print_plt and not drone_soln) : plt.show()
    return dist

def plot_drones(csv_file, nodes, launchpts, print_plt, save_plt, w_d, color = 'r'):
    routes_node = []
    # Read CSV file and extract data
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        header = next(reader)
        # Extract list of node ID's from each row as seperate routes
        routes_node = [[int(node) for node in row if node != ''] for row in reader]

    routes_pt = []  # Dictionary to store the x, y coordinates of each node in the route
    dist_d_total = 0
    # Plot drones on the existing plot
    for route in routes_node:
        dist_route = 0
        route_coords = []
        x, y = launchpts[route[0]]
        route_coords.append([route[0], x, y])

        for pt in range(1,len(route)-2):
            x, y = nodes[route[pt]-1][0], nodes[route[pt]-1][1]
            route_coords.append([route[pt], x, y])
            dist_leg = w_d * Dist(route_coords[-1][1:], route_coords[-2][1:])
            print(dist_leg)
            dist_route += dist_leg
        x, y = launchpts[route[-2]]
        route_coords.append([route[-2], x, y])
        dist_leg = w_d * Dist(route_coords[-1][1:], route_coords[-2][1:])
        print(dist_leg)
        dist_route += dist_leg
        print(f"Drone dist: {dist_route:.2f}\n")
        dist_d_total += dist_route
        # # # # ignore return to launch point # x, y = launchpts[route[-1]] # route_coords.append([route[-1], x, y])
        routes_pt.append(route_coords)
        plt.plot([x[1] for x in route_coords], [y[2] for y in route_coords], color, linestyle='-', linewidth=1)
    print(f"\nTotal drone distance travelled: {dist_d_total:.2f}\n")
    print(f"\nTotal MS distance travelled: {dist_total:.2f}\n")
    plt.annotate(f'D_dist = {dist_d_total:.2f}\n@w_d = {w_d}', xy=(0.3, 1), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = color, fontweight='bold')
    plt.annotate(f'TOTAL =\n{(dist_d_total+dist_total):.2f}', xy=(0.65, 1), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = 'b', fontweight='bold')
    print(f"\nTotal ms+d distance travelled: {(dist_d_total+dist_total):.2f}\n")

    for i,I in enumerate(routes_pt):
        print("Vehicle: ",i)
        for j,J in enumerate(I):
            print(j,'\t\t',routes_pt[i][j])

    # if (csv_file.find('NN')): plt.annotate(f'NN = {dist}', xy=(0.01, 1), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = color, fontweight='bold')
    # elif (csv_file.find('Gd')): plt.annotate(f'Gd = {dist}', xy=(0.01, 0.95), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = color, fontweight='bold')

    # annotate plot with cluster path
    plt.annotate(f'DronePath = {d_path}', xy=(0.01, -0.1), xycoords='axes fraction', ha='left', va='top')##, fontsize=12, fontweight='bold')

    if (save_plt):        # Save the plot in folder "plots" with the current date as the filename
        if not os.path.exists("plots"): # Create the "plots" folder if it doesn't exist
            os.makedirs("plots")
            if not os.path.exists("plots/d_routing"): # Create the "plots" folder if it doesn't exist
                os.makedirs("plots/d_routing")
        current_datetime = datetime.datetime.today().strftime('%Y%m%d_%H-%M-%S')
        plt.savefig(f"plots/d_routing/{current_datetime}.png")
    if print_plt : plt.show()
    return dist_total

dist_total = 0

if cluster_soln: nodes = plot_clusters(      cluster_path, print_plt, save_plt)

print("\nNo clusters: "+str(max([sublist[-1] for sublist in nodes])+1))

if ms_soln: 
    if (max([sublist[-1] for sublist in nodes])>0):
        launchpts = get_launchpts(launchpts_path)     # launchpts returns a dictionary of launch points by ID = (x, y)
    # c_gd = 'k'
    dist_total +=       w_ms *  plot_MS_route(ms_path, print_plt, save_plt, w_ms)
    # c_nn = 'k'
    # dist_drones_nn = plot_MS_route('ms_route/24-03-14_15-32-32 ms_launch_route_NN.csv', c_nn)
    # plt.annotate(f'Drones_dist_NN = {dist_drones_nn}', xy=(0.01, 0.85), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = c_gd, fontweight='bold')

if drone_soln: 
    # c_d_gd = 'r'
    dist_total +=       w_d *   plot_drones(d_path, nodes, launchpts, print_plt, save_plt, w_d)

print("End of Plot_clusters.py")