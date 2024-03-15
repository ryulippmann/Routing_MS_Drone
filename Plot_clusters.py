import csv
import matplotlib.pyplot as plt
from collections import OrderedDict
import datetime
import os

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

    # zip the x, y coordinates and cluster IDs together
    # nodes = list(zip(cluster_ids, x, y))

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

    plt.title('Reefs Clustered by Cluster ID')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc='upper right')    # Show the legend in top right corner
    plt.grid(True)

    if (print_plt) : plt.show()
    if (save_plt):        # Save the plot in folder "plots" with the current date as the filename
        if not os.path.exists("plots"): # Create the "plots" folder if it doesn't exist
            os.makedirs("plots")
            if not os.path.exists("plots/clustering"): # Create the "plots" folder if it doesn't exist
                os.makedirs("plots/clustering")
        current_datetime = datetime.datetime.today().strftime('%Y%m%d_%H-%M-%S')
        plt.savefig(f"plots/clustering/{current_datetime}__kM={kmeans_iters}.png")
    return nodes

def plot_MS_route(csv_file, color):
    # Read CSV file and extract data
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        data = list(reader)

    dist = float(reader._fieldnames[3])

    # Extract x, y coordinates from the data
    x = [float(row['X']) for row in data]
    y = [float(row['Y']) for row in data]
    
    # Annotate the plot with the coordinates of the first point as "depot"
    plt.annotate(f'DEPOT: ({x[0]}, {y[0]})', xy=(x[0], y[0]), xycoords='data', ha='left', va='top', fontsize=14, color='black')

    # Plot MS route on the existing plot
    plt.plot(x, y, color, linestyle='-', linewidth=2)  # You can adjust color, linestyle, and linewidth as needed
    # plt.show()
    return dist

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

    return launchpts

def plot_drones(csv_file, nodes, launchpts, color = 'k'):
    routes_node = []
    # Read CSV file and extract data
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        header = next(reader)
        # Extract list of node ID's from each row as seperate routes
        routes_node = [[int(node) for node in row if node != ''] for row in reader]

    routes_pt = []  # Dictionary to store the x, y coordinates of each node in the route
    # Plot drones on the existing plot
    for route in routes_node:
        route_coords = []
        x, y = launchpts[route[0]]
        route_coords.append([route[0], x, y])

        for pt in range(1,len(route)-2):
            x, y = nodes[route[pt]-1][0], nodes[route[pt]-1][1]
            route_coords.append([route[pt], x, y])
        x, y = launchpts[route[-2]]
        route_coords.append([route[-2], x, y])
        # # # # ignore return to launch point
        # x, y = launchpts[route[-1]]
        # route_coords.append([route[-1], x, y])
        routes_pt.append(route_coords)

        plt.plot([x[1] for x in route_coords], [y[2] for y in route_coords], color, linestyle='-', linewidth=1)  # You can adjust color, linestyle, and linewidth as needed
        # (x, y, color, linestyle='-', linewidth=1)  # You can adjust color, linestyle, and linewidth as needed
        # plt.show()
        # plt.plot(x, y, color, linestyle='-', linewidth=1)  # You can adjust color, linestyle, and linewidth as needed

    # # Plot drones on the existing plot
    # for i in range(len(x)):
    #     plt.annotate(f'{launchID[i]}', xy=(x[i], y[i]), xycoords='data', ha='left', va='bottom', fontsize=8, color=color, fontweight='bold')
    # plt.scatter(x, y, color=color, label='Drones')

    # return plt
        
    for i,I in enumerate(routes_pt):
        print("Vehicle: ",i)
        for j,J in enumerate(I):
            print(j,'\t\t',routes_pt[i][j])
    return 0

print_plt = False
save_plt = True

nodes = plot_clusters(      'clusters/24-03-15_11-03-35 clusters_init.csv', print_plt, save_plt)

# c_nn = 'k'
# dist_nn = plot_MS_route('ms_route/24-03-14_15-32-32 ms_launch_route_NN.csv', c_nn)
# plt.annotate(f'NN = {dist_nn}', xy=(0.01, 0.90), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = c_nn, fontweight='bold')

# c_gd = 'r'
# # d='24-03-15_10-30-58'
# # d='24-03-15_10-32-25'
# d='24-03-15_10-33-09'
# dist_gd = plot_MS_route(    str('ms_route/' + d + ' ms_launch_route_fullSoln.csv'), c_gd)
# plt.annotate(f'Gd = {dist_gd}', xy=(0.01, 0.95), xycoords='axes fraction', ha='left', va='top', fontsize=12, color = c_gd, fontweight='bold')

# launchpts = get_launchpts(  str('launchPts/'+ d + ' launchPts_fullSoln.csv'))     # launchpts returns a dictionary of launch points by ID = (x, y)

# dist_drones = plot_drones(  str('d_route/'  + d + ' drone_route_list.csv'), nodes, launchpts)

# current_datetime = datetime.datetime.today().strftime('%Y%m%d_%H-%M-%S')
# if save_plt: plt.savefig(f"plots/ms_routing/{current_datetime}.png")

# plt.show()

print("End of Plot_clusters.py")


# import csv
# import matplotlib.pyplot as plt
# 
# def plot_clusters_repeatColors(csv_file):
#     # Read the CSV file
#     with open(csv_file, 'r') as file:
#         reader = csv.DictReader(file)
#         data = {field: [] for field in reader.fieldnames}
#         for row in reader:
#             for field, value in row.items():
#                 data[field].append(value)
# 
#     # Plot points colored by cluster ID
#     cluster_ids = set(data['ClusterID'])
#     colors = plt.cm.rainbow([i / len(cluster_ids) for i in range(len(cluster_ids))])
# 
#     for cluster_id, color in zip(cluster_ids, colors):
#         x = [float(data['X'][i]) for i in range(len(data['ClusterID'])) if data['ClusterID'][i] == cluster_id]
#         y = [float(data['Y'][i]) for i in range(len(data['ClusterID'])) if data['ClusterID'][i] == cluster_id]
#         plt.scatter(x, y, color=color, label=f'Cluster {cluster_id}')
# 
#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.title('Reefs Clustered by Cluster ID')
#     plt.legend()
#     plt.show()
# 
# # Example usage:
# plot_clusters_repeatColors('clusters.csv')
