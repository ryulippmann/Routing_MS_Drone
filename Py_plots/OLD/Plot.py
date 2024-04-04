import csv
import matplotlib.pyplot as plt
import datetime
import os

# Lists to store x and y coordinates
id_name, x_coords,y_coords, visited = [], [], [], []


pt_filename = 'points'
# rt_filename = '23-08-03_12-37-02 seq_rand_route_list'
# rt_filename = 'routes/23-08-07_10-51-35 gd_route_list'
rt_filename = 'routes/23-08-07_10-52-05 multi_sa_route_list'
# ms_rt_filename = 'ms_routes/23-08-07_10-52-35 ms_init_route_list'
# ms_rt_filename = 'ms_routes/23-08-07_11-12-46 ms_gd_route_list'
# ms_rt_filename = 'ms_routes/23-08-07_11-15-15 ms_opt_stops-sa_route_list'
ms_rt_filename = 'ms_routes/23-08-07_11-15-20 ms_sa_route_list'

print_plt = False

# Read POINTS file
with open(pt_filename+'.csv', 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)           # Read the header row

    for row in csv_reader:
        id_name.append(int(row[0]))
        x_coords.append(float(row[1]))
        y_coords.append(float(row[2]))
        visited.append(int(row[3]))

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

# Read MS_ROUTES file
if 'ms_rt_filename' in locals():
    with open(ms_rt_filename+'.csv', 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)           # Read the header row
        ms_routes = []
        for row in enumerate(csv_reader):
            # ms_routes.append([])
            for stop in range(len(row[1])-1):
                ms_routes.append([int(row[1][stop])])
else: ms_routes=[]

# Increase the size of the plot to prevent title cutoff
plt.figure(figsize=(8,7))  # You can adjust the width and height as needed

## DRONES ##
# Print to terminal
# Include point coords
try:
    for i,I in enumerate(routes):
        print("Vehicle: ",i)
        for j,J in enumerate(I):
            routes[i][j].append([x_coords[J[0]],y_coords[J[0]]])
            print(j,'\t\t',routes[i][j])

    # Plot ROUTES
    for tour in range(len(routes)):
        plt.plot([routes[tour][i][1][0] for i in range(len(routes[tour]))], [routes[tour][i][1][1] for i in range(len(routes[tour]))], '-o')
except:
    # for i,I in enumerate(routes):
    #     print("Vehicle: ",i)
    for j,J in enumerate(ms_routes):
        routes.append([x_coords[J[0]],y_coords[J[0]]])
        print(j,'\t\t',routes[i][j])
    # Plot ROUTES
    for tour in range(len(routes)):
        plt.plot([routes[tour][i][1][0] for i in range(len(routes[tour]))], [routes[tour][i][1][1] for i in range(len(routes[tour]))], '-o')


## MOTHERSHIPS ##
# Print to terminal
# Include point coords
print("MOTHERSHIP:")
for j,J in enumerate(ms_routes):
    ms_routes[j].append([x_coords[J[0]],y_coords[J[0]]])
    print(j,'\t\t',ms_routes[j])

# Plot MOTHERSHIP ROUTES
# for tour in range(len(routes)):
plt.plot([ms_routes[i][1][0] for i in range(len(ms_routes))], [ms_routes[i][1][1] for i in range(len(ms_routes))], '-ko')

# Plot POINTS
for p,P in enumerate(visited):
    if (P==0):
        plt.scatter(x_coords[p], y_coords[p], s=50, c='black', marker='x')

# Set the plot limits
plt.xlim(0, 100)
plt.ylim(0, 100)

# Annotate the points with numbers
for (i, x, y) in zip(id_name, x_coords, y_coords): plt.annotate(str(i), (x, y), textcoords="offset points", xytext=(0, 10), ha='center')

plt.xlabel('X')
plt.ylabel('Y')
if 'ms_rt_filename' in locals():
    if "gd" in ms_rt_filename:
        plt.title('Greedy plot of \n{:s}\n{:s}'.format(rt_filename, ms_rt_filename))
    elif "opt_stops" in ms_rt_filename:
        plt.title('Ms_opt_stops plot of \n{:s}\n{:s}'.format(rt_filename, ms_rt_filename))
    else:
        plt.title('Plot of \n{:s}'.format(rt_filename, ms_rt_filename))
elif 'rt_filename' in locals():
    if "gd" in rt_filename:
        plt.title('Greedy plot of \n{:s}'.format(rt_filename))
    elif "sa" in rt_filename:
        plt.title('Sim_anneal plot of \n{:s}'.format(rt_filename))
    else:
        plt.title('Plot of \n{:s}\n{:s}'.format(rt_filename))
else: plt.title('Plot of ...')

try:
    plt.text(0.95, -10, "{:d} reefs\n{:d}  tours\n{:d}  m/s stops incl start & finish".format(len(id_name),len(routes),len(ms_routes)))
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
