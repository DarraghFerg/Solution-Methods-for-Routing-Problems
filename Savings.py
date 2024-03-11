import numpy as np
import matplotlib.pyplot as plt
import re
from gurobipy import *
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

File = r"c:\Users\s2026970\Desktop\instances_VRP_draft_limits\r_25_70_30_70_1.txt"          #File to extract data from
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
ports_and_depot = [0] + ports                                                   #List of enumerated ports and depot, beginning with depot
arcs = [(i, j) for i in ports_and_depot for j in ports_and_depot]               #List of all arcs in problem, between ports i and j for i and j in list of ports and depot described above
vehicles = range(6)                                                             #Range over number of vehicles in the problem
I_max = distance_matrix.shape[0]  



def clarke_wright_algorithm(customers, depot):
    savings = []
    for i in range(len(customers)):
        for j in range(i + 1, len(customers)):

            customer_i = customers[i]
            customer_j = customers[j]
            
            distance_i_depot = calculate_distance(customer_i['x'], customer_i['y'], depot['x'], depot['y'])
            distance_j_depot = calculate_distance(customer_j['x'], customer_j['y'], depot['x'], depot['y'])
            distance_i_j = calculate_distance(customer_i['x'], customer_i['y'], customer_j['x'], customer_j['y'])

            saving = (distance_i_depot + distance_j_depot - distance_i_j)
            savings.append((customer_i['customer_id'], customer_j['customer_id'], saving))

    savings.sort(key=lambda x: x[2], reverse=True)

    routes = [[customer['customer_id']] for customer in customers]
    for saving in savings:
        customer_i_id, customer_j_id, saving_value = saving
        route_i_index = find_route_index(routes, customer_i_id)
        route_j_index = find_route_index(routes, customer_j_id)

        if route_i_index != route_j_index:
            route_i = routes[route_i_index]
            route_j = routes[route_j_index]

            if is_merge_feasible(route_i, route_j, customers, depot):
                merged_route = merge_routes(route_i, route_j)
                routes.remove(route_i)
                routes.remove(route_j)
                routes.append(merged_route)

    return routes
 

 
def calculate_distance(x1, y1, x2, y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
 
 

def find_route_index(routes, customer_id):

    for i, route in enumerate(routes):
        if customer_id in route:
            return i
        
    return -1



def load_at_i(route_i, route_j, customers, i):

    merged_route = route_i + route_j
    i_index = merged_route.index(i) if i in merged_route else None
    load = 0
    if i_index is not None:
        for node in range(len(merged_route)):
            if node > i_index:
                load += q[merged_route[node]][0]
    return load 

    


def load_at_i_constraint(route_i, route_j, customers):

    merged_route = route_i + route_j
    for i_index in range(len(merged_route)):
        i = merged_route[i_index]
        next_customer_id = merged_route[i_index + 1] if i_index + 1 < len(merged_route) else None
        
        if next_customer_id is None:
            return True
        
        load_at_port_i = sum(q[customer_id - 1][0] for customer_id in merged_route if customer_id >= i_index)
        load_at_port_j = sum(q[customer_id - 1][0] for customer_id in merged_route if customer_id >= next_customer_id)
        if load_at_port_j >= load_at_port_i + q[i][0]:
            return False  
    return True 



def load_i(route, customers, i):
    load_at_port_i = 0
    found_i = False  # Flag to indicate if i is found in the route
    
    for node in route:
        if node == i:
            found_i = True
        elif found_i:
            load_at_port_i += q[node][0]  # Assuming q is a dictionary with node as key
            
    return load_at_port_i                   #if port > i_index then we want to find the corresponding route value for port



def load_i_constraint(route):
    merged_route = route
    for i_index in range(len(merged_route)):
        i = merged_route[i_index]
        next_customer_id = merged_route[i_index + 1] if i_index + 1 < len(merged_route) else None
        
        if next_customer_id is None:
            return True
        
        load_at_port_i = sum(q[customer_id - 1][0] for customer_id in merged_route if customer_id >= i_index)
        load_at_port_j = sum(q[customer_id - 1][0] for customer_id in merged_route if customer_id >= next_customer_id)
        if load_at_port_j >= load_at_port_i + q[i][0]:
            return False  
    return True 
 


def is_merge_feasible(route_i, route_j, customers, depot):

    for i in route_i+route_j:
        if sum([customers[customer_id - 1]['demand'] for customer_id in route_i + route_j]) <= Q[0][5]:
            if load_at_i(route_i, route_j, customers, i) <= Q[0][5]:
                if load_at_i(route_i, route_j, customers, i) <= max_safe_capacity[0][5]:
                    return True
        #else:
        #    return False
 
 

def merge_routes(route_i, route_j):
    return route_i + route_j
 


customers = [{'customer_id': port, 'demand': q[port][0], 'x': coordinates[port][0], 'y': coordinates[port][1]} for port in ports]

depot = {'x': coordinates[0][0], 'y': coordinates[0][1], 'capacity': q[0][0]}

start_time = time.time()
routes_ = clarke_wright_algorithm(customers, depot)
 
routes = []
for route in routes_:
    routes.append([0] + route + [0])

# Print the resulting routes
for i, route in enumerate(routes):
    print(f"Route {i+1}: {route}")



vehicle_capacities = [(vehicle, capacity) for vehicle, capacity in enumerate(Q[0])]



def distance_in_route(route):
    distance = 0
    previous_customer = None

    for customer_id in route:

        if customer_id == route[0]:
            distance += distance_matrix[0][customer_id]

        if customer_id == route[-1]:
            distance += distance_matrix[customer_id][0]

        if previous_customer is not None:
            distance += distance_matrix[customer_id][previous_customer]

        previous_customer = customer_id
    return distance



def demand_in_route(route):
    demand = 0
    for customer_id in route:
        demand += q[customer_id][0]
    return demand



def route_objective(vehicle, route):
    value = 0
    for customer_id in route:
        value += entry_cost[customer_id-1][vehicle]
    value += cost[0][vehicle]*distance_in_route(route)
    return value

def objective_value(vehicle, route):
    total = 0
    for route in routes:
        total += route_objective(vehicle, route)
    return total


def sort_routes_by_value(route_list):
    sorted_routes = sorted(route_list, key=lambda x: x[1], reverse=True)
    return sorted_routes



def assign_vehicles(routes):

    '''if route 1 demands are satisfied by vehicle 1 assign vehicle 1 to route 1, otherwise try 2... remove assigned route from list of routes and vehicle 1 from list of vehicles'''
    assignment = []
    distances = []

    for i, route in enumerate(routes):
        distances.append((route, distance_in_route(route)))
    distances = sort_routes_by_value(distances)

    assigned_routes = set()
    assigned_vehicles = set()


    while len(assignment) < len(routes): 
        for vehicle in vehicles: 
            if vehicle in assigned_vehicles:
                continue    
            for i, (route, value) in enumerate(distances):
                if i in assigned_routes:
                    continue
                if demand_in_route(route) <= Q[0][vehicle]:
                    if load_i(route, customers, i) <= Q[0][vehicle]:
                        assignment.append((f'Route {i}, Vehicle {vehicle}'))
                        assigned_routes.add(i)
                        assigned_vehicles.add(vehicle)
                        break
            else:
                continue  
            break 
    return assignment

print(assign_vehicles(routes))
end_time = time.time()

Obj = 0
for assignment_item in assign_vehicles(routes):
    route, vehicle = assignment_item.split(', ')
    route = int(route.split(' ')[1])
    vehicle = int(vehicle.split(' ')[1])
    Obj += route_objective(vehicle, routes[route])
print(f'Objective Value: {Obj}')
print(end_time-start_time)