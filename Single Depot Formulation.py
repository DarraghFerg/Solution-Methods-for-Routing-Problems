import numpy as np
import matplotlib.pyplot as plt
import re
from gurobipy import *
import random

def Read_File(filename):                                                                    #Function to read the data from the file into python
    with open(filename, 'r') as file:
        content = file.read()                                                               #Open the file in read mode

    data = {}                                                                               #Initiate data as empty list

                                                                                            #Each file has number of inputs under which data is stored as a matrix
    numerical_data = re.findall(r'\[([\d\s.]+)\]', content)                                 #Extract values following each header as numerical values and remove '\t's and '\n's

    for i, section in enumerate(['Q', 'q', 'L', 'coord', 'c', 'r']):                        #Parsing the numerical data into lists of integers or floats
        data[section] = []
        lines = numerical_data[i].strip().split('\n')
        for line in lines:
            values = line.strip().split()  # Split using spaces
            if section == 'c':
                data[section].append([float(val) for val in values])
            elif section == 'coord':
                data[section].append((int(values[0]), int(values[1])))
            else:
                data[section].append([float(val) if '.' in val else int(val) for val in values])

    return data

def calculate_distance(coordinates, i, j):                                                  #Function that takes the coordinates of two points given in file, assigns to variables the x and y coordinates of given points and finds the square euclidean distance between them
    """
    Calculate the Euclidean distance between two points.
    """
    x_i, y_i = coordinates[i][0], coordinates[i][1]                                     
    x_j, y_j = coordinates[j][0], coordinates[j][1]


    return np.sqrt((x_i - x_j) ** 2 + (y_i - y_j) ** 2)

def calculate_distance_matrix(coordinates):                                                 #Function that creates a matrix of distances for every pair of ports given in data
    num_points = len(coordinates)                                                           #Initiate matrix as square matrix of zeros with number of columns and rows equal to the number of ports in the data
    distance_matrix = np.zeros((num_points, num_points))                                    

    for i in range(num_points):
        for j in range(num_points):
            distance_matrix[i, j] = calculate_distance(coordinates, i, j)                   #Loop over all pairs of ports and depot in the data and assign their pair wise euclidean distance to their associated element of the distance matrix

    return distance_matrix

def nearest_port(i):
    distance_from_i_to_ports = []
    for j in range(distance_matrix.shape[0]):
        if i != j:
            distance_from_i_to_ports.append((j, distance_matrix[i][j]))
    distance_from_i_to_ports.sort(key=lambda x: x[1])
    return distance_from_i_to_ports[0][1] 

File = r"c:\Users\s2026970\Desktop\instances_VRP_draft_limits\r_15_70_30_70_10.txt"          #File to extract data from
Input_Data = Read_File(File)                                                                #Store data as a callable variable

### Parameters ###

Q = Input_Data["Q"]                                                             #Capacity of each ship
q = Input_Data["q"]                                                             #Demand of each port
max_safe_capacity = Input_Data["L"]                                             #Load of ship required for safe access to port decided from linear draft function
coordinates = Input_Data["coord"]                                               #Coordinates of each port and depot initialised at (0, 0)
cost = Input_Data["c"]                                                          #Hourly cost of operating ship in Euros
entry_cost = Input_Data["r"]                                                    #One time entry fee per ship for port 

distance_matrix = calculate_distance_matrix(coordinates)                        #Subscriptable variable storing the Euclidean distance matrix
ports = [i for i in range(1, distance_matrix.shape[0])]                         #List of enumerated ports
depots = [0, 17]
ports_and_depot = depots + ports
arcs = [(i, j) for i in ports_and_depot for j in ports_and_depot]               #List of all arcs in problem, between ports i and j for i and j in list of ports and depot described above
vehicles_at_0 = range(3)
vehicles_at_17 = range(3)                                                             #Range over number of vehicles in the problem
I_max = distance_matrix.shape[0]                                                #Number of total ports in problem

###
print(q)
q.append(17)
print(q)
### Model ###

m = Model("MDCVRP")


### Variables ###.

x = m.addVars(arcs, vehicles, vtype=GRB.BINARY, name = 'x')                                             #x binary variable equal to 1 if arc (i, j) is traversed by ship s
y = m.addVars(ports, vehicles, vtype=GRB.BINARY, name = 'y')                                            #y binary variable equal to 1 if port i is served by ship s
u = m.addVars(ports, lb = 0, ub = I_max,  vtype = GRB.CONTINUOUS, name = 'u')                           #u variable taking values in the natural numbers to track position of ports in a ships journey
l = m.addVars(ports_and_depot, vehicles, lb = 0, ub = 100000, vtype=GRB.CONTINUOUS, name = 'l')         #l variable tracking load on ship at each port

###

### Optimization ###
m.ModelSense = GRB.MINIMIZE                                                                             #Objective is to minimize cost of travel

###

### Objective Value ###

m.setObjective(quicksum(x[i, j, s]*distance_matrix[i][j]*cost[0][s] for i, j in arcs for s in vehicles) + quicksum(entry_cost[i-1][s]*y[i, s] for i in ports for s in vehicles))

###

### Constraints ###

m.addConstrs((quicksum(y[i, s] for s in vehicles) == 1 for i in ports), name = 'Assign port to ship')
m.addConstrs((quicksum(q[i][0] * y[i, s] for i in ports) <= Q[0][s] for s in vehicles), name = 'Max load capacity')
m.addConstrs((quicksum(x[i, j, s] for i in ports_and_depot) == y[j, s] for j in ports for s in vehicles), name = 'Visit a port once')
m.addConstrs((quicksum(x[i, j, s] for i in ports_and_depot) == quicksum(x[j, i, s] for i in ports_and_depot) for j in ports for s in vehicles), name = 'Leave port once')
m.addConstrs((quicksum(x[0, j, s] for j in ports) <= len(vehicles) for s in vehicles), name = 'Subtour')                                                                                            #Each ship must enter and exit the depot only once if assigned to a port

m.addConstrs(((u[j] >= u[i] + 1 - I_max * (1-quicksum(x[i, j, s] for s in vehicles)) for i in ports for j in ports)), name = 'Track position')
m.addConstrs((l[j, s] >= l[i, s] - q[i][0] - Q[0][s] * (1 - x[i, j, s]) for i in ports for j in ports_and_depot for s in vehicles), name = 'Track load')
m.addConstrs((l[i, s] <= max_safe_capacity[i-1][s] for i in ports for s in vehicles), name = 'Draft Limit')
m.addConstrs((l[0, s] == quicksum(q[i][0] * y[i, s] for i in ports) for s in vehicles), name = 'Sum of demands at depot')

###

### Valid Inequalities ###
'''#Valid Inequality 1:
Total_Demand = 0
for i in ports:
    Total_Demand += q[i][0]
for i in ports:
    for j in ports:
        for s in vehicles:
            x[0, j, s] <= 1-(1/Total_Demand) * (l[0, s]- max_safe_capacity[i-1][s])

#Valid Inequality 2:
for i in ports:
    for j in ports: 
        for s in vehicles:
            if (q[i][0] + q[j][0] > max_safe_capacity[i-1][s]):
                x[i, j, s] == 0'''

#Valid Inequality 3:
for i in ports:
    q_big = max(q[i])
    
T = m.addVars(ports_and_depot, vehicles, vtype = GRB.CONTINUOUS, name = 'T')
for i in ports:
    for s in vehicles:
        l[0, s] - q_big*(u[i]-1) <= (max_safe_capacity[i-1][s] + Q[0][s] * (1-y[i, s]))

### Optimization Parameters ###
m.write('myLP3-1.lp')

m.Params.TimeLimit=15
m.Params.MIPFocus=2
m.Params.MIPGap = 0.1

m.optimize()

'''

iter = 1
max_iter = 10
Objectives = []
Bounds = []
while iter <= max_iter:
    m.optimize()
    Objectives.append(m.ObjVal)
    Bounds.append(m.ObjBound)
    print(Bounds)
    m.reset()
    print(iter)
    iter += 1
    
m.update()
print(sum(Objectives)/10)

# Convert each number to a string and join them with '+'
Objectives = ' + '.join(map(str, Objectives))
Bounds = ' + '.join(map(str, Bounds))

print(Objectives)
print(min(Bounds))'''

'''eps = 1

iter_count = 1
itermax = 10

no_improve = 0
no_improve_max = 2

best_solution = x
best_obj = m.ObjVal

while iter_count <= itermax and no_improve <= no_improve_max:
    print(iter_count)
    print(no_improve)
    IR = []
    current_solution = best_solution
    m_rand = 5
    ports_selected = random.sample(ports, m_rand)
    #random.seed(42)
    IR = ports_selected
    I_IR = [i for i in ports if not i in IR]
    alpha = 1.5

    Ports_within_proximity = []

    for i in IR:
        for j in I_IR:
            for s in vehicles:
                if distance_matrix[i][j] <= alpha * nearest_port(i):
                    Ports_within_proximity.append(j)
    IR.extend(Ports_within_proximity)

    for i in IR:
        for j in range(distance_matrix.shape[0]):
            for s in vehicles:
                x[i, j, s] = 0
                x[j, i, s] = 0
          


    for i in ports_and_depot:
        for j in range(distance_matrix.shape[0]):
            for s in vehicles:
                if x[i, j, s].X > 0.99:
                    x[i, j, s] = 1

                if x[j, i, s].X > 0.99:
                    x[j, i, s] = 1

    m.update()
    m.write('model.sol')
    m.Params.BestObjStop <= best_obj
    m.update()
    m.read('model.sol')
    m.update()
    m.optimize()
    obj_new = m.ObjVal
    m.update()

    if obj_new < best_obj:
        best_solution = x
        no_improve = 0
    else:
        no_improve += 1

    iter_count += 1


if m.status == GRB.INFEASIBLE:
        m.computeIIS()
        m.write("infeasible.lp")
        print('\nThe following constraint(s) cannot be satisfied:')
        for c in m.getConstrs():
            if c.IISConstr:
                print('%s' % c.constrName)'''

'''points = [(key[0], key[1]) for key, value in sol_x.items() if value > 0]

for route in points:
    route_coordinates = [Input_Data["coord"][i] for i in route]

    x_coords = [point[0] for point in route_coordinates]
    y_coords = [point[1] for point in route_coordinates]

    # Plot points
    plt.scatter(x_coords, y_coords, color='red')

    # Plot route
    plt.plot(x_coords, y_coords, color='blue')

    # Add labels
    for i, (x, y) in enumerate(route_coordinates):
        plt.text(x, y, f'{route[i]}', fontsize=12, ha='center', va='bottom')

# Plot depot as a square
depot_x, depot_y = 0, 0
plt.scatter(depot_x, depot_y, color='green', marker='s', label='Depot')

# Set plot title and labels
plt.title('Network Route')
plt.xlabel(r'$x$ Coordinate')
plt.ylabel(r'$y$ Coordinate')
plt.legend()

# Show plot
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

#m.write("myLP.mst")                                                                                            #Write MIP solution to text file for checking'''