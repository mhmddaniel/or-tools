from flask import request, jsonify
from app.api.v1 import v1_blueprint
from app.services.geocoding import geocode_address, reverse_geocode
from app.utils.helpers import validate_lat_lon

@v1_blueprint.route('/geocode', methods=['GET'])
def geocoding():
    address = request.args.get('address')
    if not address:
        return jsonify({"status": False, "message": "address parameter is required"}), 400
    
    try:
        coord = geocode_address(address)
        return jsonify({"coordinate": coord}), 200
    except Exception as e:
        return jsonify({"status": False, "message": "error processing geocoding", "error": str(e)}), 500

@v1_blueprint.route('/geocode/reverse', methods=['GET'])
def georeverse():
    coordinate = request.args.get('coordinate')
    if not coordinate:
        return jsonify({"status": False, "message": "coordinate parameter is required"}), 400
    
    if not validate_lat_lon(coordinate):
        return jsonify({"status": False, "message": "coordinate format is invalid"}), 400

    try:
        address = reverse_geocode(coordinate)
        return jsonify({"address": address}), 200
    except Exception as e:
        return jsonify({"status": False, "message": "error processing reverse geocoding", "error": str(e)}), 500
