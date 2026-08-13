import copy
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from app.utils.distance import compute_distance_matrix, compute_ovrp_distance_matrix
from app.utils.helpers import create_demand_model
from app.services.geocoding import create_location_model

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
            cur_distance = routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            if route_distance != 0:
                cur_distance = cur_distance - data.get('slack', 20000)
            route_distance += cur_distance
            cur_route.append(cur_distance)
        rd.append(route_distance)
        rd_matrix.append(cur_route)
    return [rd, rd_matrix]

def get_solution(data, manager, routing, solution, show_depot):
    route = []
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        sub_route = []
        while not routing.IsEnd(index):
            sub_route.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
        sub_route.append(manager.IndexToNode(index))
        if not show_depot:
            sub_route = sub_route[1:-1]
        route.append(sub_route)
    return route

def get_dropped(data, manager, routing, solution, show_depot):
    dropped = []
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped.append(manager.IndexToNode(node))
    return dropped

def solve_routing(result: dict) -> dict:
    debug_mode = False
    data = {}

    data['num_vehicles'] = result.get('number_of_vehicles', 1)
    
    if 'vehicle_capacities' in result:
        data['vehicle_capacities'] = result['vehicle_capacities']
    else:
        data['vehicle_capacities'] = [0] * data['num_vehicles']

    if 'vehicle_weight' in result:
        data['vehicle_weight'] = result['vehicle_weight']
    else:
        data['vehicle_weight'] = [0] * data['num_vehicles']

    time_limit = result.get('time_limit', 8)
    slack = int(result.get('slack_time', 20)) * 1000
    use_all_routes = result.get('use_all_routes', 0)
    back_to_depot = result.get('back_to_depot', 1)
    multitrip = result.get('multitrip', 0)
    maximum_distance = result.get('maximum_distance', 0)
    maximum_task = result.get('maximum_task', 0)
    time_windows = result.get('time_windows', [])
    vehicle_time_windows = result.get('vehicle_time_windows', [])
    vehicle_tags = result.get('vehicle_tags', [])
    
    if not vehicle_time_windows:
        vehicle_time_windows = [[0, 100000]] * data['num_vehicles']
        
    if not vehicle_tags:
        vehicle_tags = [[] for _ in range(data['num_vehicles'])]
    
    if multitrip == 1:
        back_to_depot = 1
        max_capacity = max(result.get("vehicle_capacities", [0]))
        dummy_depot = copy.deepcopy(result['task_list'][0])
        dummy_depot["capacity_demand"] = -max_capacity
        for _ in range(5):
            result['task_list'].append(copy.deepcopy(dummy_depot))
            
    if back_to_depot == 0:
        result['task_list'].append(copy.deepcopy(result['task_list'][0]))
        
    data['slack'] = slack
    capacity_demand = create_demand_model(result, "capacity_demand")
    weight_demand = create_demand_model(result, "weight_demand")
    locations = create_location_model(result)
    
    # Parse service times
    service_times = []
    for task in result['task_list']:
        service_times.append(int(task.get('service_time', 0)))
    data['service_times'] = service_times
    
    if back_to_depot == 0:
        data['starts'] = [(len(result['task_list']) - 1) for _ in range(data['num_vehicles'])]
        data['ends'] = [0 for _ in range(data['num_vehicles'])]
        data['distance_matrix'], data['time_matrix'] = compute_ovrp_distance_matrix(locations, slack)
    else:
        data['distance_matrix'], data['time_matrix'] = compute_distance_matrix(locations, slack)

    used_all = False
    def_time_limit = time_limit
    total_routes = data['num_vehicles']
    state = -1
    solution_data = {}

    while not used_all:
        data['depot'] = 0
        distance_limit = time_limit * 30 * 1000
        
        if maximum_distance > 0:
            md_val = maximum_distance * 1000
            if md_val < distance_limit:
                distance_limit = copy.deepcopy(md_val)

        if back_to_depot == 0:
            manager = pywrapcp.RoutingIndexManager(
                len(data['distance_matrix']), data['num_vehicles'], data['starts'], data['ends'])
        else:
            manager = pywrapcp.RoutingIndexManager(
                len(data['distance_matrix']), data['num_vehicles'], data['depot'])
                
        routing = pywrapcp.RoutingModel(manager)

        # Vehicle specific matching logic (existing)
        for index_task in range(len(result['task_list'])):
            if 'vehicle_list' in result['task_list'][index_task]:
                vl = result['task_list'][index_task]['vehicle_list']
                if len(vl) > 0:
                    vl.insert(0, -1)
                    index = manager.NodeToIndex(index_task)
                    routing.VehicleVar(index).SetValues(vl)
                    
        # Tag/Skill based matching logic (Mile.app feature)
        for index_task in range(1, len(result['task_list'])):
            task_tags = result['task_list'][index_task].get('tags', [])
            if task_tags:
                allowed_vehicles = [-1] # -1 allows node to be dropped if necessary
                for v_id in range(data['num_vehicles']):
                    v_tags = vehicle_tags[v_id]
                    if any(t in v_tags for t in task_tags):
                        allowed_vehicles.append(v_id)
                index = manager.NodeToIndex(index_task)
                routing.VehicleVar(index).SetValues(allowed_vehicles)

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

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        weight_callback_index = routing.RegisterUnaryTransitCallback(weight_callback)

        if len(time_windows) > 0 or len(vehicle_time_windows) > 0:
            def time_callback(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                travel_time = data['time_matrix'][from_node][to_node]
                service_time = data['service_times'][from_node]
                return travel_time + service_time

            time_callback_index = routing.RegisterTransitCallback(time_callback)
            routing.AddDimension(
                time_callback_index,
                30,  # allow waiting time
                100000,  # maximum time per vehicle
                False,  # Don't force start cumul to zero.
                'Time')
            time_dimension = routing.GetDimensionOrDie('Time')
            
            # Add time windows for tasks
            if len(time_windows) > 0:
                for location_idx, time_window in enumerate(time_windows):
                    if location_idx == 0:
                        continue
                    index = manager.NodeToIndex(location_idx)
                    time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
            
            # Add vehicle time windows (working hours)
            for vehicle_id in range(data['num_vehicles']):
                index = routing.Start(vehicle_id)
                vtw = vehicle_time_windows[vehicle_id]
                time_dimension.CumulVar(index).SetRange(vtw[0], vtw[1])
                # Cap the End node
                end_index = routing.End(vehicle_id)
                time_dimension.CumulVar(end_index).SetRange(vtw[0], vtw[1])
                
            for i in range(data['num_vehicles']):
                routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
                routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

        if maximum_task > 0:
            plus_one_callback_index = routing.RegisterUnaryTransitCallback(lambda index: 1)
            routing.AddDimension(plus_one_callback_index, 0, maximum_task+1, True, 'Counter')
            counter_dimension = routing.GetDimensionOrDie('Counter')
            for vehicle_id in range(data['num_vehicles']):
                index = routing.End(vehicle_id)
                counter_dimension.CumulVar(index).SetRange(0, maximum_task+1)

        if multitrip == 1:
            routing.AddDimensionWithVehicleCapacity(
                demand_callback_index, 0, data['vehicle_capacities'], True, 'Capacity')
            capacity_dimension = routing.GetDimensionOrDie('Capacity')
            len_task = len(result['task_list'])
            for node_index in range(len_task - 5, len_task):
                index = manager.NodeToIndex(node_index)
                capacity_dimension.SlackVar(index).SetRange(0, max_capacity)
                routing.AddDisjunction([index], 0)
        else:
            routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, data['vehicle_capacities'], True, 'Capacity')
            routing.AddDimensionWithVehicleCapacity(weight_callback_index, 0, data['vehicle_weight'], True, 'Weight')

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.time_limit.seconds = 60
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC)

        routing.AddDimension(transit_callback_index, slack, distance_limit, True, 'Distance')

        for node in range(1, len(data['distance_matrix'])):
            loc = locations[manager.NodeToIndex(node)]
            if loc == ('0', '0') or loc == (0, 0):
                penalty = 0
            else:
                penalty = distance_limit * 100
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

        solution = routing.SolveWithParameters(search_parameters)
        
        if solution:
            solution_data['routing'] = get_solution(data, manager, routing, solution, False)
            solution_data['dropped'] = get_dropped(data, manager, routing, solution, False)
            distance_data = get_distance(data, manager, routing, solution)
            solution_data['distance'] = distance_data[0]
            solution_data['distance_array'] = distance_data[1]
        else:
            solution_data['routing'] = []
            solution_data['dropped'] = []
            solution_data['distance'] = []
            solution_data['distance_array'] = []

        solution_data["gelocation"] = locations
        solution_data["capacity_demand"] = capacity_demand
        solution_data["weight_demand"] = weight_demand
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
        else:
            used_all = True

    if back_to_depot == 0:
        solution_data["gelocation"].pop()
        solution_data["capacity_demand"].pop()

    if multitrip == 1:
        for _ in range(5):
            solution_data["gelocation"].pop()
            solution_data["capacity_demand"].pop()
        
        if solution_data.get("dropped"):
            filtered_dropped = [task for task in solution_data["dropped"] if task < (len(result['task_list']) - 5)]
            solution_data["dropped"] = filtered_dropped

        if solution_data.get("routing"):
            filtered_routing = []
            for route in solution_data["routing"]:
                filtered_task = []
                for task in route:
                    if task < (len(result['task_list']) - 5):
                        filtered_task.append(task)
                    else:
                        filtered_task.append(0)
                filtered_routing.append(filtered_task)
            solution_data["routing"] = filtered_routing

    return solution_data
