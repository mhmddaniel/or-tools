from __future__ import print_function
import flask
import math
import geopy
import json
import re
from geopy import distance
from geopy.geocoders import Here
import os
import warnings
import sys
import pickle
import json
from flask_cors import CORS
from flask import Flask,make_response,render_template,redirect,url_for,request, jsonify
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

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
    capacity_demand         = create_demand_model(result)
    locations               = create_location_model(result)

    data['distance_matrix'] = compute_distance_matrix (locations)
    
    if debug_mode==True:
        print ("POST:","\n")
        print (json.dumps(result),"\n")
        print ("locations:","\n")
        print (json.dumps(locations),"\n")
        print ("distance_matrix:","\n")
        print (json.dumps(data['distance_matrix']),"\n")

    try:
        data['num_vehicles'] = result['number_of_vehicles']
    except AttributeError:
        data['num_vehicles'] = 1

    try : 
        data['vehicle_capacities'] = result['vehicle_capacities']
    except AttributeError:
        data['vehicle_capacities'] = []

    data['depot'] = 0
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return capacity_demand[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_callback_index,0,data['vehicle_capacities'],True,'Capacity')

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds      = 60
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    routing.AddDimension(transit_callback_index,20,400000,True,'Distance')

    #add penalty rule to able nodes to be dropped
    for node in range(1, len(data['distance_matrix'])):
        #set 0 penalty if place is not found, this should allow solution to drop 
        if locations[manager.NodeToIndex(node)]==('0','0') or locations[manager.NodeToIndex(node)]==(0,0):
            penalty = 0
        else :
            penalty = 500000
        #print (penalty)    
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # distance_dimension = routing.GetDimensionOrDie('Distance')
    # distance_dimension.SetGlobalSpanCostCoefficient(100)
    solution = routing.SolveWithParameters(search_parameters)
    solution_data = {}

    if solution:
        solution_data['routing'] = get_solution(data, manager, routing, solution,False)
        solution_data['dropped'] = get_dropped(data, manager, routing, solution,False)
    else:
        solution_data['routing'] = []
    solution_data["gelocation"]         = locations
    solution_data["capacity_demand"]    = capacity_demand
    solution_data["vehicle_capacities"] = data['vehicle_capacities']

    return json.dumps(solution_data)

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
            geolocator = Here("Pe8yMxfGgxyn9yWsRDU9","OrUmfFZvhbc2KNVLpvffrw")
            address = clean_address(task["address"])
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
            print (task["address"]) 
            print('{"lat":"',t[0],'","lon":"',t[1],'"},')    
            location.append(t)
    return location

def create_demand_model(data):
    task_list = data['task_list']
    capacity_demand = []
    for index,task in enumerate(task_list):
        try : task["capacity_demand"]
        except KeyError: 
            task["capacity_demand"] = 0
        finally:   
            capacity_demand.append(task["capacity_demand"])
    return capacity_demand

def compute_distance_matrix(locations):
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                distances[from_counter][to_counter] = int (round(distance.great_circle(from_node,to_node).km *1000))
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
#app.run()        
    app.run(host='165.22.49.16',port='80')