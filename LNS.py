import numpy as np
import matplotlib.pyplot as plt
import re
from gurobipy import *
import time

File = r"c:\Users\s2026970\Desktop\instances_VRP_draft_limits\r_25_70_30_70_6navi_9.txt"          #File to extract data from

def Read_File(filename):
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

Input_Data = Read_File(File)

def calculate_distance(coordinates, i, j):                                              #Function that takes the coordinates of two points given in file, assigns to variables the x and y coordinates of given points and finds the square euclidean distance between them
    """
    Calculate the Euclidean distance between two points.
    """
    x_i, y_i = coordinates[i][0], coordinates[i][1]
    x_j, y_j = coordinates[j][0], coordinates[j][1]


    return np.sqrt((x_i - x_j) ** 2 + (y_i - y_j) ** 2)

def calculate_distance_matrix(coordinates):                                                 #Function that creates a matrix of distances for every pair of points
    num_points = len(coordinates)
    distance_matrix = np.zeros((num_points, num_points))

    for i in range(num_points):
        for j in range(num_points):
            distance_matrix[i, j] = calculate_distance(coordinates, i, j)

    return distance_matrix

def nearest_port(i):
    distance_from_i_to_ports = []
    for j in range(distance_matrix.shape[0]):
        if i != j:
            distance_from_i_to_ports.append((j, distance_matrix[i][j]))
    distance_from_i_to_ports.sort(key=lambda x: x[1])
    return distance_from_i_to_ports[0][1]

### Parameters ###

###

Q = Input_Data["Q"]
q = Input_Data["q"]
max_safe_capacity = Input_Data["L"]
coordinates = Input_Data["coord"]
cost = Input_Data["c"]
entry_cost = Input_Data["r"]
coordinates = Input_Data["coord"]

distance_matrix = calculate_distance_matrix(coordinates)
ports = [i for i in range(1, distance_matrix.shape[0])]
ports_and_depot = [0] + ports
arcs = [(i, j) for i in ports_and_depot for j in ports_and_depot]
vehicles = range(6)
I_max = distance_matrix.shape[0]

###

### Model ###
params = {
"WLSACCESSID": 'e9790daf-2d93-4784-a738-3547d2e40a19',
"WLSSECRET": 'f5cd35b2-8f5e-43c6-9791-d955a009a61c',
"LICENSEID": 2481443
}
env = Env(params=params)

# Create the model within the Gurobi environment
m = Model("CVRP", env = env)

###

### Variables ###.

x = m.addVars(arcs, vehicles, vtype=GRB.BINARY, name = 'x')
y = m.addVars(ports, vehicles, vtype=GRB.BINARY, name = 'y')
u = m.addVars(ports, lb = 0, ub = I_max,  vtype = GRB.CONTINUOUS, name = 'u')
l = m.addVars(ports_and_depot, vehicles, lb = 0, ub = 100000, vtype=GRB.CONTINUOUS, name = 'l')

###

### Optimization ###
m.ModelSense = GRB.MINIMIZE

###

### Objective Value ###

m.setObjective(quicksum(x[i, j, s]*distance_matrix[i][j]*cost[0][s] for i, j in arcs for s in vehicles) + quicksum(entry_cost[i-1][s]*y[i, s] for i in ports for s in vehicles))

###

### Constraints ###

m.addConstrs((quicksum(y[i, s] for s in vehicles) == 1 for i in ports), name = 'Assign port to ship')
m.addConstrs((quicksum(q[i][0] * y[i, s] for i in ports) <= Q[0][s] for s in vehicles), name = 'Max load capacity')
m.addConstrs((quicksum(x[i, j, s] for i in ports_and_depot) == y[j, s] for j in ports for s in vehicles), name = 'Visit a port once')
m.addConstrs((quicksum(x[i, j, s] for i in ports_and_depot) == quicksum(x[j, i, s] for i in ports_and_depot) for j in ports for s in vehicles), name = 'Leave port once')
m.addConstrs(quicksum(x[0, j, s] for j in ports) <= len(vehicles) for s in vehicles)

m.addConstrs(((u[j] >= u[i] + 1 - I_max * (1-quicksum(x[i, j, s] for s in vehicles)) for i in ports for j in ports)), name = 'Track position')
m.addConstrs((l[j, s] >= l[i, s] - q[i][0] - Q[0][s] * (1 - x[i, j, s]) for i in ports for j in ports_and_depot for s in vehicles), name = 'Track load')
m.addConstrs((l[i, s] <= max_safe_capacity[i-1][s] for i in ports for s in vehicles), name = 'Draft Limit')
m.addConstrs((l[0, s] == quicksum(q[i][0] * y[i, s] for i in ports) for s in vehicles), name = 'Sum of demands at depot')

##
### Valid Inequalities ###
#Valid Inequality 3:
for i in ports:
    q_big = max(q[i])
    
T = m.addVars(ports_and_depot, vehicles, vtype = GRB.CONTINUOUS, name = 'T')
for i in ports:
    for s in vehicles:
        l[0, s] - q_big*(u[i]-1) <= (max_safe_capacity[i-1][s] + Q[0][s] * (1-y[i, s]))
            
###
### Optimization Parameters ###
                
m.Params.MIPGap = 0.1
m.Params.MIPFocus = 2
m.Params.TimeLimit = 15
m.update()

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