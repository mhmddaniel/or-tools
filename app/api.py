
from __future__ import print_function
import flask
import math
import geopy
import json
import re
import os
import copy
import warnings
import sys
import pickle
import json
from geopy import distance
from geopy.geocoders import Here
from flask_cors import CORS
from flask import Flask,make_response,render_template,redirect,url_for,request, jsonify
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import copy
import requests

app = flask.Flask(__name__)
app.config["DEBUG"] = True

@app.after_request
def after_request(response):
    response.headers.add("Access-Control-Allow-Origin", "*")
    response.headers.add("Access-Control-Allow-Methods", "GET,HEAD,OPTIONS,POST,PUT")
    response.headers.add("Access-Control-Allow-Headers", "*")
    return response

@app.route('/', methods=['GET'])
def home():
    return "<h1>Automatic Routing</h1><p>This engine is a prototype API for automatic routing.</p>"

@app.route('/routing', methods=['POST'])
def routing():
    debug_mode              = False
    data                    = {}
    result                  = request.get_json(force=True)

    try :
        data['num_vehicles'] = result['number_of_vehicles']
    except AttributeError:
        data['num_vehicles'] = 1

    try : 
        data['vehicle_capacities'] = result['vehicle_capacities']
    except KeyError:
        data['vehicle_capacities'] = []
        for i in range (data['num_vehicles']):
            data['vehicle_capacities'].append(0)

    try : 
        data['vehicle_weight'] = result['vehicle_weight']
    except KeyError:
        data['vehicle_weight'] = []
        for i in range (data['num_vehicles']):
            data['vehicle_weight'].append(0)

    try : 
        time_limit = result['time_limit']
    except KeyError:
        #time limit is in hours
        time_limit = 8

    try : 
        #slack is additional minutes on each stop
        slack = int(result['slack_time']) * 1000
    except KeyError:
        slack = 20000

    try : 
        #flag to balance load between worker or not
        use_all_routes = result['use_all_routes']
    except KeyError:
        use_all_routes = 0

    try : 
        #flag to change algorithm from VRP (back to depot) to OVRP (not back to depot)
        back_to_depot = result['back_to_depot']
    except KeyError:
        back_to_depot = 1

    try : 
        #flag to change algorithm from VRP (single delivery) to CVRP (with capacity and multiple trip to depot)
        multitrip = result['multitrip']
    except KeyError:
        multitrip = 0

    try : 
        #flag to change algorithm from VRP (single delivery) to CVRP (with capacity and multiple trip to depot)
        maximum_distance = result['maximum_distance']
    except KeyError:
        maximum_distance = 0

    try : 
        #flag to change algorithm from VRP (single delivery) to CVRP (with capacity and multiple trip to depot)
        maximum_task = result['maximum_task']
    except KeyError:
        maximum_task = 0

    try : 
        #flag to change algorithm from VRP (single delivery) to CVRP (with capacity and multiple trip to depot)
        time_windows = result['time_windows']
    except KeyError:
        time_windows = []

    try : 
        data['vehicle_weight'] = result['vehicle_weight']
    except KeyError:
        data['vehicle_weight'] = []
        for i in range (data['num_vehicles']):
            data['vehicle_weight'].append(0)
    
    
    if multitrip == 1:

        back_to_depot = 1 #force to use back to depot if multitrip

        max_capacity = max(result["vehicle_capacities"])
        dummy_depot = copy.deepcopy(result['task_list'][0])
        dummy_depot["capacity_demand"] = -max_capacity

        result['task_list'].insert(len(result['task_list']),dummy_depot) #duplicate depot for reloading 1
        result['task_list'].insert(len(result['task_list']),dummy_depot) #duplicate depot for reloading 2
        result['task_list'].insert(len(result['task_list']),dummy_depot) #duplicate depot for reloading 3
        result['task_list'].insert(len(result['task_list']),dummy_depot) #duplicate depot for reloading 4
        result['task_list'].insert(len(result['task_list']),dummy_depot) #duplicate depot for reloading 5
        
    if back_to_depot == 0:
        result['task_list'].insert(len(result['task_list']),result['task_list'][0]) #create dummy depot 

    data['slack'] = slack

    capacity_demand         = create_demand_model(result,"capacity_demand")
    weight_demand         = create_demand_model(result,"weight_demand")
    locations               = create_location_model(result)

    if back_to_depot == 0:
        data['starts'] = [ (len(result['task_list']) - 1) for i in range(data['num_vehicles']) ]
        data['ends'] = [ 0 for i in range(data['num_vehicles']) ]
        data['distance_matrix'] = compute_ovrp_distance_matrix (locations,slack)
    else:
        data['distance_matrix'] = compute_distance_matrix (locations,slack)

    used_all = False
    def_time_limit = time_limit

    total_routes = data['num_vehicles']
    state = -1

    while used_all == False:

        if debug_mode==True:
            print ("POST:","\n")
            print (json.dumps(result),"\n")
            print ("locations:","\n")
            print (json.dumps(locations),"\n")
            print ("distance_matrix:","\n")
            print (json.dumps(data['distance_matrix']),"\n")

        data['depot']  = 0
        distance_limit = time_limit * 30 * 1000 #asumsi speed 30 km/Jam

        if maximum_distance > 0:
            maximum_distance = maximum_distance * 1000
            if maximum_distance < distance_limit:
                distance_limit = copy.deepcopy(maximum_distance)

        # Create the routing index manager.

        if back_to_depot == 0:
            manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                data['num_vehicles'], data['starts'],
                data['ends'])
        elif back_to_depot==1:
            manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                data['num_vehicles'], data['depot'])

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Define road block
        for index_task in range(len(result['task_list'])):
            if('vehicle_list' in result['task_list'][index_task]):
                if(len(result['task_list'][index_task]['vehicle_list']) > 0):
                    result['task_list'][index_task]['vehicle_list'].insert(0,-1)
                    index = manager.NodeToIndex(index_task)
                    routing.VehicleVar(index).SetValues(result['task_list'][index_task]['vehicle_list'])

        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['distance_matrix'][from_node][to_node]

        def demand_callback(from_index):
            from_node = manager.IndexToNode(from_index)
            return capacity_demand[from_node]
        
        def weight_callback(from_index):
            from_node = manager.IndexToNode(from_index)
            return weight_demand[from_node]

        if len(time_windows) > 0:
            def time_callback(from_index, to_index):
                """Returns the travel time between the two nodes."""
                # Convert from routing variable Index to time matrix NodeIndex.
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return data['time_matrix'][from_node][to_node]

            transit_callback_index = routing.RegisterTransitCallback(time_callback)
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

            time = 'Time'
            routing.AddDimension(
                transit_callback_index,
                30,  # allow waiting time
                30,  # maximum time per vehicle
                False,  # Don't force start cumul to zero.
                time)
            time_dimension = routing.GetDimensionOrDie(time)
            # Add time window constraints for each location except depot.
            for location_idx, time_window in enumerate(time_windows):
                if location_idx == 0:
                    continue
                index = manager.NodeToIndex(location_idx)
                time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
            # Add time window constraints for each vehicle start node.
            for vehicle_id in range(data['num_vehicles']):
                index = routing.Start(vehicle_id)
                time_dimension.CumulVar(index).SetRange(time_windows[0][0],
                                                        time_windows[0][1])
            for i in range(data['num_vehicles']):
                routing.AddVariableMinimizedByFinalizer(
                    time_dimension.CumulVar(routing.Start(i)))
                routing.AddVariableMinimizedByFinalizer(
                    time_dimension.CumulVar(routing.End(i)))

        if maximum_task > 0:
            plus_one_callback_index = routing.RegisterUnaryTransitCallback(lambda index : 1)
            dimension_name = 'Counter'
            routing.AddDimension(
                plus_one_callback_index,
                0,  # null capacity slack
                maximum_task+1,  # vehicle maximum capacities
                True,  # start cumul to zero
                'Counter')
            counter_dimension = routing.GetDimensionOrDie(dimension_name)
            for vehicle_id in range(data['num_vehicles']):
                index = routing.End(vehicle_id)
                counter_dimension.CumulVar(index).SetRange(0, maximum_task+1)

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        weight_callback_index = routing.RegisterUnaryTransitCallback(weight_callback)

        if multitrip == 1:
            capacity = 'Capacity'
            routing.AddDimensionWithVehicleCapacity(
                demand_callback_index,
                0,  # Null slack
                data['vehicle_capacities'],
                True,  # start cumul to zero
                capacity)
            capacity_dimension = routing.GetDimensionOrDie(capacity)
            len_task = len(result['task_list'])
            
            print(result["vehicle_capacities"],"capacities",max_capacity)
            
            for node_index in [len_task-5, len_task-4, len_task-3, len_task-2, len_task-1]:
                index = manager.NodeToIndex(node_index)
                capacity_dimension.SlackVar(index).SetRange(0, max_capacity)
                routing.AddDisjunction([index], 0)
        else:
            routing.AddDimensionWithVehicleCapacity(demand_callback_index,0,data['vehicle_capacities'],True,'Capacity')
            routing.AddDimensionWithVehicleCapacity(weight_callback_index,0,data['vehicle_weight'],True,'Weight')

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.time_limit.seconds      = 60
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC)

        routing.AddDimension(transit_callback_index,slack,distance_limit,True,'Distance')

        #add penalty rule to able nodes to be dropped
        for node in range(1, len(data['distance_matrix'])):
            #set 0 penalty if place is not found, this should allow solution to drop 
            if locations[manager.NodeToIndex(node)] ==('0','0') or locations[manager.NodeToIndex(node)]==(0,0):
                penalty = 0
            else :
                penalty = distance_limit *100
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

        distance_dimension = routing.GetDimensionOrDie('Distance')
        # distance_dimension.SetGlobalSpanCostCoefficient(100)
        solution = routing.SolveWithParameters(search_parameters)
        solution_data = {}

        if solution:
            solution_data['routing']  = get_solution(data, manager, routing, solution,False)
            solution_data['dropped']  = get_dropped(data, manager, routing, solution,False)
            distance_data = get_distance(data, manager, routing, solution)
            solution_data['distance'] = distance_data[0]
            solution_data['distance_array'] = distance_data[1]
        else:
            solution_data['routing'] = []
            solution_data['dropped'] = []
            solution_data['distance'] = []
            solution_data['distance_array'] = []

        solution_data["gelocation"]         = locations
        solution_data["capacity_demand"]    = capacity_demand
        solution_data["weight_demand"]    = weight_demand
        solution_data["vehicle_capacities"] = data['vehicle_capacities']
        solution_data["vehicle_weight"] = data['vehicle_weight']

        empty_route_count = solution_data['routing'].count([])
        
        if use_all_routes == 0:
            used_all = True
        elif empty_route_count > 0:
            if empty_route_count <= total_routes:
                total_routes = empty_route_count
            else:
                if state == -1:
                    state = 1
                elif state == 1:
                    state = -1
            time_limit += state
            if time_limit > 24:
                used_all = True
            if time_limit == 0:
               time_limit = def_time_limit + 1
        else :
            used_all = True

    # print_solution(data,manager,routing,solution)

    if back_to_depot == 0:
        solution_data["gelocation"].pop((len(result['task_list']) - 1))
        solution_data["capacity_demand"].pop((len(result['task_list']) - 1))

    if multitrip == 1:
        solution_data["gelocation"].pop() #remove dummy depot 5
        solution_data["gelocation"].pop() #remove dummy depot 4
        solution_data["gelocation"].pop() #remove dummy depot 3
        solution_data["gelocation"].pop() #remove dummy depot 2
        solution_data["gelocation"].pop() #remove dummy depot 1
        solution_data["capacity_demand"].pop() #remove dummy depot 5
        solution_data["capacity_demand"].pop() #remove dummy depot 4
        solution_data["capacity_demand"].pop() #remove dummy depot 3
        solution_data["capacity_demand"].pop() #remove dummy depot 2
        solution_data["capacity_demand"].pop() #remove dummy depot 1
        
        if(solution_data["dropped"]):
            filtered_dropped = [task for task in solution_data["dropped"] if task < (len(result['task_list'])-5)]
            solution_data["dropped"] = filtered_dropped

        if(solution_data["routing"]):
            filtered_routing = []
            for route in solution_data["routing"]:
                filtered_task = []
                for task in route:
                    if task < (len(result['task_list'])-5):
                        filtered_task.append(task)
                    else:
                        filtered_task.append(0)
                filtered_routing.append(filtered_task)
            solution_data["routing"] = filtered_routing


    return json.dumps(solution_data)

@app.route('/time-estimated', methods=['POST'])
def time_estimated():
    result                  = request.get_json(force=True)
    data                    = {}

    try :
        data['coordinate_origin'] = result['coordinate_origin']
    except KeyError:
        response = {}
        response["status"] = False
        response["message"] = "coordinate origin is required"
        response["request"] = result
        return json.dumps(response)
    
    if(not validate_lat_lon(data['coordinate_origin'])):
        response = {}
        response["status"] = False
        response["message"] = "coordinate origin format is invalid"
        response["request"] = result
        return json.dumps(response)
    
    try :
        data['coordinate_destination'] = result['coordinate_destination']
    except KeyError:
        response = {}
        response["status"] = False
        response["message"] = "coordinate destination is required"
        response["request"] = result
        return json.dumps(response)

    if(not validate_lat_lon(data['coordinate_destination'])):
        response = {}
        response["status"] = False
        response["message"] = "coordinate destination format is invalid"
        response["request"] = result
        return json.dumps(response)

    try :
        data['speed'] = result['speed']
    except KeyError:
        data['speed'] = 30

    origin  = data['coordinate_origin'].split(",")
    data['coordinate_origin'] = str(origin[1])+","+str(origin[0])

    destination  = data['coordinate_destination'].split(",")
    data['coordinate_destination'] = str(destination[1])+","+str(destination[0])

    distance_value = json.loads(get_distance_osrm(data['coordinate_origin'],data['coordinate_destination']))

    data["distance"] = round(distance_value["routes"][0]["distance"])/1000

    data["travel_time"] = round((data["distance"] / data["speed"]) * 60)
    
    return json.dumps(data)

def get_distance_osrm(coordinate_origin,coordinate_destination):
    url = "https://osrm.mile.app/route/v1/driving/"+str(coordinate_origin)+";"+str(coordinate_destination)

    payload = {}
    headers = {
    'Cookie': '__cfduid=d01f83999b0989b042da492ba589083001569464335'
    }

    response = requests.request("GET", url, headers=headers, data = payload)

    return response.text

@app.route('/geocoding', methods=['POST'])
def geocoding():
    result                  = request.get_json(force=True)
    data                    = {}
    
    try :
        data['address'] = result['address']
    except KeyError:
        response = {}
        response["status"] = False
        response["message"] = "address is required"
        response["request"] = result
        return json.dumps(response)

    t="0,0"
    geolocator = Here(app_id="brUMhDedQJcTEBv3WRm2",app_code="gEuqige5xqyAJ3Gi5x1Qaw") 
    #geolocator = Here("Pe8yMxfGgxyn9yWsRDU9","OrUmfFZvhbc2KNVLpvffrw") #old auth
    address    = clean_address(data["address"])
    glocation  = geolocator.geocode(address)
    #check if lat lon is available, else move it to the antartic zone :)
    try : glocation.address
    except KeyError : 
        t = "0,0"
    else :
        t = str(glocation.latitude)+","+str(glocation.longitude)

    data["coordinate"] = t
    return json.dumps(data)

@app.route('/georeverse', methods=['POST'])
def georeverse():
    result                  = request.get_json(force=True)
    data                    = {}
    
    try :
        data['coordinate'] = result['coordinate']
    except AttributeError:
        response = {}
        response["status"] = False
        response["message"] = "coordinate is required"
        response["request"] = result
        return json.dumps(response)
    
    if(not validate_lat_lon(data['coordinate'])):
        response = {}
        response["status"] = False
        response["message"] = "coordinate format is invalid"
        response["request"] = result
        return json.dumps(response)

    t=""
    geolocator = Here(app_id="brUMhDedQJcTEBv3WRm2",app_code="gEuqige5xqyAJ3Gi5x1Qaw") 
    #geolocator = Here("Pe8yMxfGgxyn9yWsRDU9","OrUmfFZvhbc2KNVLpvffrw") #old auth
    glocation  = geolocator.reverse(query=data["coordinate"])
    #check if lat lon is available, else move it to the antartic zone :)

    try : glocation.address
    except AttributeError : 
        t = ""
    else :
        t = glocation.address

    data["address"] = t
    return json.dumps(data)

def create_location_model(data):
    task_list = data['task_list']
    geopy.geocoders.options.default_timeout = 15
    location = []
    #not_found = 0
    for index,task in enumerate(task_list):
        t=(0,0)
        try : task["lat"]
        # lat not given, try reversing address
        except KeyError: 
            geolocator = Here("brUMhDedQJcTEBv3WRm2","gEuqige5xqyAJ3Gi5x1Qaw") 
            #geolocator = Here("Pe8yMxfGgxyn9yWsRDU9","OrUmfFZvhbc2KNVLpvffrw") #old auth
            address    = clean_address(task["address"])
            glocation  = geolocator.geocode(address)
            #check if lat lon is available, else move it to the antartic zone :)
            try : glocation.address
            except AttributeError : 
                t = (0,0)
            else :
                t = (glocation.latitude,glocation.longitude)
        else :
            t = (task["lat"],task["lon"])
            task["address"] = index
        finally:
            # print (task["address"]) 
            # print('{"lat":"',t[0],'","lon":"',t[1],'"},')    
            location.append(t)
    return location

def create_demand_model(data,demand):
    task_list       = data['task_list']
    arr_demand = []
    for index,task in enumerate(task_list):
        try : task[demand]
        except KeyError: 
            task[demand] = 0
        finally:   
            arr_demand.append(task[demand])
    return arr_demand

def compute_distance_matrix(locations,slack):
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            # get_distance_osrm(from_node,to_node)
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                distances[from_counter][to_counter] = int (round(distance.great_circle(from_node,to_node).km *1000)+slack)
    return distances

def compute_ovrp_distance_matrix(locations,slack):
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            # get_distance_osrm(from_node,to_node)
            if from_counter == 0 or to_counter ==0:
                distances[from_counter][to_counter] = 0
            elif from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                distances[from_counter][to_counter] = int (round(distance.great_circle(from_node,to_node).km *1000)+slack)
    return distances

def print_solution(data, manager, routing, solution):
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0

        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))

def get_distance(data, manager, routing, solution):
    rd = []
    rd_matrix = []
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_distance = 0
        cur_route = []
        while not routing.IsEnd(index):
            cur_distance = 0
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            cur_distance = routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            if (route_distance != 0):
                cur_distance = cur_distance - data['slack']
            route_distance += cur_distance
            cur_route.append(cur_distance)
        rd.append(route_distance)
        rd_matrix.append(cur_route)
    return [rd,rd_matrix]

def get_solution(data, manager, routing, solution,show_depot):
    route=[]
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        sub_route=[]
        while not routing.IsEnd(index):
            sub_route.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
        sub_route.append(manager.IndexToNode(index))
        if show_depot==False:
            sub_route = sub_route[1:-1]
        route.append(sub_route)
    return route

def get_dropped(data, manager, routing, solution,show_depot):
    dropped=[]
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            #dropped_nodes += ' {}'.format(manager.IndexToNode(node))
            dropped.append(manager.IndexToNode(node))
    return dropped

def clean_address(address_string):
    words = ["kel","kec","kelurahan","kecamatan","indonesia","jl","jalan","kota","samping","rumah","sakit","rs","no","nomor"," rt","rw","idn","blok"]
    new_string = address_string.lower()
    new_string = re.sub(r'[^a-z]+', ' ', new_string)
    for word in words:
        new_string = new_string.replace(word+" ", ' ')
        new_string = new_string.replace(" "+word, ' ')
    new_string = new_string + " indonesia"
    new_string = re.sub(' +',' ', new_string)
    split_word = new_string.split()
    new_string = (" ".join(sorted(set(split_word), key=split_word.index)))
    return new_string

def validate_lat_lon(str):
    x = re.search("^(-?\d+(\.\d+)?),\s*(-?\d+(\.\d+)?)$", str)
    if x:
        return True
    else:
        return False