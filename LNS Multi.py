import numpy as np
import matplotlib.pyplot as plt
import re
from gurobipy import *
import random
import time

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

File = r"c:\Users\s2026970\Desktop\instances_VRP_draft_limits\r_15_30_30_30_1.txt"          #File to extract data from
Input_Data = Read_File(File)                                                                #Store data as a callable variable

### Parameters ###

Q = Input_Data["Q"]       
Q.append([30000, 50000, 100000])                                                      #Capacity of each ship
q = Input_Data["q"]  
q.append([0])                                                           #Demand of each port
max_safe_capacity = Input_Data["L"]                                             #Load of ship required for safe access to port decided from linear draft function
coordinates = Input_Data["coord"]   
coordinates.append((100, 100))                                            #Coordinates of each port and depot initialised at (0, 0)
cost = Input_Data["c"] 
cost.append([1.0, 1.3, 2.0])                                                        #Hourly cost of operating ship in Euros
entry_cost = Input_Data["r"]                                                    #One time entry fee per ship for port 

distance_matrix = calculate_distance_matrix(coordinates)                        #Subscriptable variable storing the Euclidean distance matrix

ports = [i for i in range(2, distance_matrix.shape[0])]                         #List of enumerated ports
depots = [0, 1]
ports_and_depot = depots + ports
arcs = [(i, j) for i in ports_and_depot for j in ports_and_depot]               #List of all arcs in problem, between ports i and j for i and j in list of ports and depot described above
vehicles= range(3)
I_max = distance_matrix.shape[0]-2                                              #Number of total ports in problem
###
### Model ###

m = Model("MDCVRP")


### Variables ###.

x = m.addVars(arcs, vehicles, depots, vtype=GRB.BINARY, name = 'x')                                             #x binary variable equal to 1 if arc (i, j) is traversed by ship s
y = m.addVars(ports, vehicles, depots, vtype=GRB.BINARY, name = 'y')                                            #y binary variable equal to 1 if port i is served by ship s
u = m.addVars(ports, ub = I_max,  vtype = GRB.CONTINUOUS, name = 'u')                                           #u variable taking values in the natural numbers to track position of ports in a ships journey
l = m.addVars(ports_and_depot, vehicles, depots, lb = 0, ub = 100000, vtype=GRB.CONTINUOUS, name = 'l')         #l variable tracking load on ship at each port

###

### Optimization ###
m.ModelSense = GRB.MINIMIZE                                                                             #Objective is to minimize cost of travel

###

### Objective Value ###

m.setObjective(quicksum(x[i, j, s, p]*distance_matrix[i-1][j-1]*cost[p][s] for i, j in arcs for s in vehicles for p in depots) + quicksum(entry_cost[i-3][s]*y[i, s, p] for i in ports for s in vehicles for p in depots))

###

### Constraints ###

m.addConstrs((quicksum(y[i, s, p] for s in vehicles for p in depots) == 1 for i in ports), name = 'Assign port to ship')
m.addConstrs((quicksum(q[i-1][0] * y[i, s, p] for i in ports for p in depots) <= Q[p][s] for s in vehicles for p in depots), name = 'Max load capacity')
m.addConstrs((quicksum(x[i, j, s, p] for i in ports_and_depot) == y[j, s, p] for j in ports for s in vehicles for p in depots), name = 'Visit a port once')
m.addConstrs((quicksum(x[i, j, s, p] for i in ports_and_depot for p in depots) == quicksum(x[j, i, s, p] for i in ports_and_depot for p in depots) for j in ports for s in vehicles), name = 'Leave port once')
#m.addConstrs((quicksum(x[0, j, s, p] for j in ports for p in depots) <= len(vehicles) for s in vehicles), name = 'Subtour')                                                                                            #Each ship must enter and exit the depot only once if assigned to a port
m.addConstrs((quicksum(x[0, j, s, p] for j in ports for p in depots) - (quicksum(x[j, 0, s, p] for j in ports for p in depots)) == 0 for s in vehicles), name = 'Subtour')                                                                                            #Each ship must enter and exit the depot only once if assigned to a port
m.addConstrs((quicksum(x[1, j, s, p] for j in ports for p in depots) - (quicksum(x[j, 1, s, p] for j in ports for p in depots)) == 0 for s in vehicles), name = 'Subtour')                                                                                            #Each ship must enter and exit the depot only once if assigned to a port

m.addConstrs(((u[j] >= u[i] + 1 - I_max * (1-quicksum(x[i, j, s, p] for s in vehicles for p in depots)) for i in ports for j in ports)), name = 'Track position')
m.addConstrs((l[j, s, p] >= l[i, s, p] - q[i-1][0] - Q[p][s] * (1 - x[i, j, s, p]) for i in ports for j in ports_and_depot for s in vehicles for p in depots), name = 'Track load')
m.addConstrs((l[i, s, p] <= max_safe_capacity[i-3][s] for i in ports for s in vehicles for p in depots), name = 'Draft Limit')
m.addConstrs((l[0, s, p] == quicksum(q[i-1][0] * y[i, s, p] for i in ports for p in depots) for s in vehicles for p in depots), name = 'Sum of demands at depot')

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

'''#Valid Inequality 3:
for i in ports:
    q_big = max(q[i])
    
T = m.addVars(ports_and_depot, vehicles, vtype = GRB.CONTINUOUS, name = 'T')
for i in ports:
    for s in vehicles:
        l[0, s] - q_big*(u[i]-1) <= (max_safe_capacity[i-1][s] + Q[0][s] * (1-y[i, s]))'''

### Optimization Parameters ###
m.write('myLP3-1.lp')

m.Params.TimeLimit=15
m.Params.MIPFocus=2
m.Params.MIPGap = 0.1

start_time = time.time()

m.optimize()
objective = m.ObjVal
x

import random
random.seed(42)

ports_selected = []
I_IR = [i for i in ports if not i in ports_selected]
new_arcs = [(i, j) for i in ports_selected for j in range(distance_matrix.shape[0])]

m.update()
m.Params.TimeLimit = 60
m.update()
iteration_counter = 1
max_itererations = 10

continue_optimization = True

def mycallback2(model, where):
    global iteration_counter
    global max_iterations
    global objective
    global new_arcs
    global ports_selected
    global continue_optimization

    if where == GRB.Callback.MIPSOL:
        if continue_optimization:        
            m_rand = 7
            alpha = 2
            ports_selected = random.sample(ports, m_rand)
            ports_in_proximity = []
            
            for i in ports_selected:
                for j in I_IR:
                    for s in vehicles:
                        if distance_matrix[i][j] <= alpha * nearest_port(i):
                            ports_in_proximity.append(j)
                            
            ports_selected.extend(ports_in_proximity)
            new_arcs = [(i, j) for i in ports_selected for j in range(distance_matrix.shape[0])]

            for i, j in new_arcs:
                for s in vehicles:
                    model.cbSetSolution(x[i, j, s], 0)
                    model.cbSetSolution(x[j, i, s], 0)
            
            for i, j, s in x:
                if model.cbGetSolution(x[i, j, s]) > 0:  
                    model.cbSetSolution(x[i, j, s], 1)
            
    elif where == GRB.Callback.MIP:
        current_obj_value = model.cbGet(GRB.Callback.MIP_OBJBST)
        
        if current_obj_value < objective:
            model.terminate()
            objective = current_obj_value

            iteration_counter += 1
            
            if iteration_counter < max_iterations:
                print(iteration_counter)
                continue_optimization = True
                model.optimize(mycallback2)
            else:
                continue_optimization = False
                
iteration_counter = 0
max_iterations = 10

m.optimize(mycallback2)
end_time=time.time()
t = end_time-start_time
print(t)