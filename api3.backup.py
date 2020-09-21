from __future__ import print_function
import flask
import math
import geopy
import json
from geopy import distance
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
#response.headers.add('Access-Control-Allow-Origin', '*')
	#response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
  	#response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    response.headers.add("Access-Control-Allow-Origin", "*")
    	#response.headers.add("Access-Control-Allow-Credentials", "true");
    response.headers.add("Access-Control-Allow-Methods", "GET,HEAD,OPTIONS,POST,PUT")
    	#response.headers.add("Access-Control-Allow-Headers", "Access-Control-Allow-Headers, Origin,Accept, X-Requested-With, Content-Type, Access-Control-Request-Method, Access-Control-Request-Headers")  	
    response.headers.add("Access-Control-Allow-Headers", "*")  	
    return response

@app.route('/', methods=['GET'])
def home():
    return "<h1>Automatic Routing</h1><p>This engine is a prototype API for automatic routing.</p>"

@app.route('/routing', methods=['POST'])

def routing():
    data = {}
    result = request.get_json(force=True)
    # print ("POST:","\n")
    # print (json.dumps(result),"\n") 
    locations = create_location_model(result)
    # print ("locations:","\n")
    # print (json.dumps(locations),"\n") 
    data['distance_matrix'] = compute_distance_matrix (locations)
    # print ("distance_matrix:","\n")
    # print (json.dumps(data['distance_matrix']),"\n")
    
    try:
        data['num_vehicles'] = result['number_of_vehicles']
    except AttributeError:
        data['num_vehicles'] = 1
    data['depot'] = 0
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        100000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)
    solution = routing.SolveWithParameters(search_parameters)
    #solution_data = make_response()

    solution_data = {}

    if solution:
        solution_data['routing'] = get_solution(data, manager, routing, solution,False)
    else:
        solution_data['routing'] = []
    #solution_data.headers.add("Access-Control-Allow-Origin", "*")
    #solution_data.headers.add("Access-Control-Allow-Headers", "*")
    #solution_data.headers.add("Access-Control-Allow-Methods", "*")
    return json.dumps(solution_data)

def create_location_model(data):
    task_list = data['task_list']
    location = []
    for task in task_list:
        t = (task["lat"],task["lon"])
        location.append(t)
    return location

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
app.run(host='165.22.49.16',port='80')
