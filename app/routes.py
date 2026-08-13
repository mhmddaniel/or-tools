from flask import Blueprint, request, jsonify
from app.services.routing import solve_routing
from app.services.geocoding import geocode_address, reverse_geocode
from app.utils.distance import get_distance_osrm
from app.utils.helpers import validate_lat_lon
import json

api_blueprint = Blueprint('api', __name__)

@api_blueprint.after_request
def after_request(response):
    response.headers.add("Access-Control-Allow-Origin", "*")
    response.headers.add("Access-Control-Allow-Methods", "GET,HEAD,OPTIONS,POST,PUT")
    response.headers.add("Access-Control-Allow-Headers", "*")
    return response

@api_blueprint.route('/', methods=['GET'])
def home():
    return "<h1>Automatic Routing</h1><p>This engine is a prototype API for automatic routing.</p>"

@api_blueprint.route('/routing', methods=['POST'])
def routing():
    result = request.get_json(force=True)
    solution_data = solve_routing(result)
    return jsonify(solution_data)

@api_blueprint.route('/time-estimated', methods=['POST'])
def time_estimated():
    result = request.get_json(force=True)
    data = {}

    if 'coordinate_origin' not in result:
        return jsonify({"status": False, "message": "coordinate origin is required", "request": result})
    
    data['coordinate_origin'] = result['coordinate_origin']
    
    if not validate_lat_lon(data['coordinate_origin']):
        return jsonify({"status": False, "message": "coordinate origin format is invalid", "request": result})
    
    if 'coordinate_destination' not in result:
        return jsonify({"status": False, "message": "coordinate destination is required", "request": result})

    data['coordinate_destination'] = result['coordinate_destination']

    if not validate_lat_lon(data['coordinate_destination']):
        return jsonify({"status": False, "message": "coordinate destination format is invalid", "request": result})

    data['speed'] = result.get('speed', 30)

    origin = data['coordinate_origin'].split(",")
    data['coordinate_origin'] = f"{origin[1]},{origin[0]}"

    destination = data['coordinate_destination'].split(",")
    data['coordinate_destination'] = f"{destination[1]},{destination[0]}"

    distance_resp = get_distance_osrm(data['coordinate_origin'], data['coordinate_destination'])
    
    try:
        distance_value = json.loads(distance_resp)
        data["distance"] = round(distance_value["routes"][0]["distance"]) / 1000
        data["travel_time"] = round((data["distance"] / data["speed"]) * 60)
    except Exception as e:
        return jsonify({"status": False, "message": "error computing distance", "error": str(e)})
    
    return jsonify(data)

@api_blueprint.route('/geocoding', methods=['POST'])
def geocoding():
    result = request.get_json(force=True)
    if 'address' not in result:
        return jsonify({"status": False, "message": "address is required", "request": result})
    
    coord = geocode_address(result['address'])
    return jsonify({"coordinate": coord})

@api_blueprint.route('/georeverse', methods=['POST'])
def georeverse():
    result = request.get_json(force=True)
    if 'coordinate' not in result:
        return jsonify({"status": False, "message": "coordinate is required", "request": result})
    
    if not validate_lat_lon(result['coordinate']):
        return jsonify({"status": False, "message": "coordinate format is invalid", "request": result})

    address = reverse_geocode(result['coordinate'])
    return jsonify({"address": address})
