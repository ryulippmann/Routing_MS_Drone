import csv
import matplotlib.pyplot as plt
import datetime
import os

path = 'outputs'
run_name = '24-06-27_13-50-30_FullRuns'

print_plt = 0
save_plt = 1

def plot_clusters(csv_file):
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        data = list(reader)

    kmeans_iters = int(reader.fieldnames[6])
    w_ms = float(reader.fieldnames[9])
    w_d = float(reader.fieldnames[12])
    total_iters = int(reader.fieldnames[14])

    x = [float(row['X']) for row in data]
    y = [float(row['Y']) for row in data]
    node_id = [int(row['ReefID']) for row in data]

    # Initialize nodes list with empty lists
    nodes = [[] for _ in range(len(node_id))]
    for n, (i, j) in enumerate(zip(x, y)):
        nodes[node_id[n] - 1] = [i, j]

    return nodes, w_ms, w_d, x, y, node_id

def get_launchpts(csv_file):
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        data = list(reader)

    launchpts = {int(row['ID']): (float(row['X']), float(row['Y'])) for row in data}
    return launchpts

def plot_MS_route(csv_file, color='k', alpha=0.2):
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        data = list(reader)

    dist = float(reader.fieldnames[3])
    x = [float(row['X']) for row in data]
    y = [float(row['Y']) for row in data]

    plt.plot(x, y, color=color, linestyle='-', linewidth=2, alpha=alpha)  # adjust color, linestyle, and linewidth as needed
    return dist

dist_total = 0

plt.figure(figsize=(12, 8))  # Initialize a single plot for all folders

solns=0
opt_path = None

# Get the list of folders matching the run_name
folders = [folder for folder in os.listdir(path) if os.path.isdir(os.path.join(path, folder))]
folders.sort(reverse=True)
idx = next((i for i, folder in enumerate(folders) if folder.startswith(run_name)), None)
# Process the 10 most recent folders
for folder in range(idx, idx-10, -1):
    folder_path = os.path.join(path, folders[folder])
    for subfolder in os.listdir(folder_path):
        subfolder_path = os.path.join(folder_path, subfolder)
        if os.path.isdir(subfolder_path):
            for sub_subfolder in os.listdir(subfolder_path):
                if sub_subfolder.endswith("_FINAL"):
                    sub_subfolder_path = os.path.join(subfolder_path, sub_subfolder)
                    if os.path.isdir(sub_subfolder_path):
                        for file in os.listdir(sub_subfolder_path):
                            if file.endswith(".csv"):
                                if file.startswith('clusters'):
                                    cluster_path = os.path.join(sub_subfolder_path, file)
                                    nodes, w_ms, w_d, x, y, node_id = plot_clusters(cluster_path)
                                if file.startswith('ms_route'):
                                    ms_path = os.path.join(sub_subfolder_path, file)
                                    ms_dist = plot_MS_route(ms_path, 'k')
                                    dist_total += ms_dist
                                    if 154.12 <= ms_dist <= 154.13:
                                        opt_path = ms_path
                                    solns += 1

# Plot folder starting with "1opt" last to ensure it is on top of all other routes. color='r' is used for this folder
if opt_path is not None:
    # dist_total +=
    plot_MS_route(opt_path, color='r', alpha=0.8)

plt.scatter(x, y, color='blue', marker='o', label='Nodes')  # Plot all nodes
avg_dist = dist_total / solns
plt.annotate(f'Average MS Distance: {avg_dist:.2f}', xy=(0.05, 0.95), xycoords='axes fraction', ha='left', va='top', fontsize=12, color='black', fontweight='bold')

plt.legend(loc='upper right')    # Show the legend in the top right corner
plt.grid(True)

if save_plt:  # Save the plot in folder "plots" with the current date as the filename
    if not os.path.exists("/figs"):  # Create the "plots" folder if it doesn't exist
        os.makedirs("/figs")
        if not os.path.exists("figs/ms_variance"):
            os.makedirs("figs/ms_variance")
    current_datetime = datetime.datetime.today().strftime('%Y%m%d_%H-%M-%S')
    plt.savefig(f"figs/ms_variance/{current_datetime}_ms_combined.png")
if print_plt:
    plt.show()

print("End of Plot_clusters.py")
