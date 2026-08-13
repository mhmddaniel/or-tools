import json
from flask import request, jsonify
from app.api.v1 import v1_blueprint
from app.utils.distance import get_distance_osrm
from app.utils.helpers import validate_lat_lon

@v1_blueprint.route('/estimations/time', methods=['POST'])
def time_estimated():
    try:
        result = request.get_json(force=True)
    except Exception:
        return jsonify({"status": False, "message": "invalid JSON payload"}), 400

    data = {}

    if 'coordinate_origin' not in result:
        return jsonify({"status": False, "message": "coordinate_origin is required", "request": result}), 400
    
    data['coordinate_origin'] = result['coordinate_origin']
    if not validate_lat_lon(data['coordinate_origin']):
        return jsonify({"status": False, "message": "coordinate_origin format is invalid", "request": result}), 400
    
    if 'coordinate_destination' not in result:
        return jsonify({"status": False, "message": "coordinate_destination is required", "request": result}), 400

    data['coordinate_destination'] = result['coordinate_destination']
    if not validate_lat_lon(data['coordinate_destination']):
        return jsonify({"status": False, "message": "coordinate_destination format is invalid", "request": result}), 400

    data['speed'] = result.get('speed', 30)

    origin = data['coordinate_origin'].split(",")
    data['coordinate_origin'] = f"{origin[1]},{origin[0]}"

    destination = data['coordinate_destination'].split(",")
    data['coordinate_destination'] = f"{destination[1]},{destination[0]}"

    try:
        distance_resp = get_distance_osrm(data['coordinate_origin'], data['coordinate_destination'])
        distance_value = json.loads(distance_resp)
        data["distance"] = round(distance_value["routes"][0]["distance"]) / 1000
        data["travel_time"] = round((data["distance"] / data["speed"]) * 60)
        return jsonify(data), 200
    except Exception as e:
        return jsonify({"status": False, "message": "error computing distance", "error": str(e)}), 500
