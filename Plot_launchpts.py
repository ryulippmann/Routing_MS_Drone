import csv
import matplotlib.pyplot as plt
import datetime
import os
import math

# Lists to store x and y coordinates
id_name, x_coords,y_coords, cluster = [], [], [], []


pt_filename = 'reef_set'
# rt_filename = '23-08-03_12-37-02 seq_rand_route_list'
# rt_filename = 'routes/23-08-07_10-51-35 gd_route_list'
rt_filename = 'drone_route_list'#drone_routes/23-11-13_08-35-12 drone_route_list - init'
# ms_rt_filename = 'ms_routes/23-08-07_10-52-35 ms_init_route_list'
# ms_rt_filename = 'ms_routes/23-08-07_11-12-46 ms_gd_route_list'
# ms_rt_filename = 'ms_routes/23-08-07_11-15-15 ms_opt_stops-sa_route_list'
ms_rt_filename = 'ms_launch_route'#ms_routes/23-11-03_14-27-56 ms_launch_route'

numClust = 10 #2
numDrones = 2
trips_clust=[]
for c in range (numClust):
    trips_clust.append(numDrones)

# update!
# number of routes in each cluster
# include start and stop nodes for each drone_route
# this needs to identify which cluster trip is in (and how many trips per cluster...)
# trips_clust = [1,2,2,1,1,1,2,1,1,1]     # randomSeed = 12345
# trips_clust = [2,2,2,2,1,1,1,1,1,2]     # ordered...
# trips_clust = [1,1,1,1,1,1,1,1,1,1]
# trips_clust = [2,2,2,2,2,2,2,2,2,2]
# trips_clust = [5,5,5,5,5,5,5,5,5,5]

print_plt = False

# Read POINTS file
with open(pt_filename+'.csv', 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)           # Read the header row

    for row in csv_reader:
        # cluster.append(int(row[0]))        
        id_name.append(int(row[1]))
        x_coords.append(float(row[2]))
        y_coords.append(float(row[3]))
        # id_name.append(int(row[0]))
        # x_coords.append(float(row[1]))
        # y_coords.append(float(row[2]))
        # cluster.append(int(row[3]))

# Read ROUTES file
if 'rt_filename' in locals():
    with open(rt_filename+'.csv', 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)           # Read the header row
        routes = []
        for row in enumerate(csv_reader):
            routes.append([])
            for stop in range(len(row[1])-1):
                routes[row[0]].append([int(row[1][stop])])
else: routes = []
routes = [route for route in routes if len(route) > 0]      # remove empty entries


# Read MS_ROUTES file
if 'ms_rt_filename' in locals():
    with open(ms_rt_filename+'.csv', 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)           # Read the header row
        ms_launch_stops = []
        for row in enumerate(csv_reader):
            # ms_routes.append([])
            # for pt in range(len(row[1])):
            try: ms_launch_stops.append([float(row[1][0]), float(row[1][1])])
            except: break
else: ms_launch_stops=[]

# Increase the size of the plot to prevent title cutoff
plt.figure(figsize=(8,7))  # You can adjust the width and height as needed

## DRONES ##
# Print to terminal
# Include point coords
# try:
for i,I in enumerate(routes):
    print("Vehicle: ",i)
    for j,J in enumerate(I):
        if (j==0 or j==len(I)-1): routes[i][j].append([ms_launch_stops[(i//numDrones)][0], ms_launch_stops[(i//numDrones)][1]])   #[x_coords[J[0]],y_coords[J[0]]])
        elif (j==len(I)-2): routes[i][j].append([ms_launch_stops[(i//numDrones)+1][0], ms_launch_stops[(i//numDrones)+1][1]])
        else: routes[i][j].append([x_coords[J[0]-1],y_coords[J[0]-1]])
        print(j,'\t\t',routes[i][j])
    # routes[i].append(['stop',[x,y]])
# except:
#     # for i,I in enumerate(routes):
#     #     print("Vehicle: ",i)
#     for j,J in enumerate(ms_launch_stops):
#         routes.append([x_coords[J[0]],y_coords[J[0]]])
#         print(j,'\t\t',routes[i][j])

z=0
for i in range(len(trips_clust)):
    # index = cluster.index(value_to_find)
    for j in range(trips_clust[i]):     # j is a proxy for number of trips in this cluster...
        routes[z][0][0]                 = '{:d} start'.format(i)
        routes[z][len(routes[z])-1][0]  = '{:d} start'.format(i)
        # routes[z].insert(0,['{:d} start'.format(i),ms_launch_stops[i]])
        routes[z][len(routes[z])-2][0]  = '{:d} stop'.format(i)
        # routes[z].append(['{:d} stop'.format(i),ms_launch_stops[i+1]])
        z+=1

# Plot ROUTES
for tour in range(len(routes)):
    plt.plot([routes[tour][i][1][0] for i in range(len(routes[tour]))], [routes[tour][i][1][1] for i in range(len(routes[tour]))], '-o')


## MOTHERSHIPS ##
# Print to terminal
# Include point coords
print("MOTHERSHIP:")
for j,J in enumerate(ms_launch_stops):
#     ms_launch_stops[j].append([J[0],J[0]])
    print(j,'\t\t',ms_launch_stops[j])

# Plot MOTHERSHIP ROUTES
# for tour in range(len(routes)):
plt.plot([ms_launch_stops[i][0] for i in range(len(ms_launch_stops))], [ms_launch_stops[i][1] for i in range(len(ms_launch_stops))], '-ko')

# Plot POINTS
# for p,P in enumerate(cluster):
#     if (P==0):
#         plt.scatter(x_coords[p], y_coords[p], s=50, c='black', marker='x')

# Set the plot limits
# plt.xlim(0, 100)
# plt.ylim(0, 100)

# Annotate the points with numbers
for (i, x, y) in zip(id_name, x_coords, y_coords): plt.annotate(str(i), (x, y), textcoords="offset points", xytext=(0, 10), ha='center')

plt.xlabel('X')
plt.ylabel('Y')
if 'ms_rt_filename' in locals():
    if "gd" in ms_rt_filename:
        plt.title('Greedy plot of \n{:s}\n{:s}\n'.format(rt_filename, ms_rt_filename))
    elif "opt_stops" in ms_rt_filename:
        plt.title('Ms_opt_stops plot of \n{:s}\n{:s}\n'.format(rt_filename, ms_rt_filename))
    else:
        plt.title('Plot of \n{:s}\n{:s}\n'.format(rt_filename, ms_rt_filename))
elif 'rt_filename' in locals():
    if "gd" in rt_filename:
        plt.title('Greedy plot of \n{:s}\n'.format(rt_filename))
    elif "sa" in rt_filename:
        plt.title('Sim_anneal plot of \n{:s}\n'.format(rt_filename))
    else:
        plt.title('Plot of \n{:s}\n'.format(rt_filename))
else: plt.title('Plot of ...')

try:
    plt.text(0.95, -10, "{:d} reefs\n{:d}  tours\n{:d}  m/s stops incl start & finish".format(len(id_name),len(routes),len(ms_launch_stops)))
except:
    plt.text(0.95, 0.95, "{:d} reefs\n".format(len(id_name)))
plt.grid(True)

######## SAVE PLOT ########
###########################
if not os.path.exists("plots"): # Create the "plots" folder if it doesn't exist
    os.makedirs("plots")

if (print_plt):
    # Save the plot in folder "plots" with the current date as the filename
    current_datetime = datetime.datetime.today().strftime('%Y-%m-%d_%H-%M-%S')
    # name_file = current_datetime + 

    plt.savefig(f"plots/{current_datetime}.png")

# Display the plot
plt.show()

print("Helloworld")
