from flask import request, jsonify
from app.api.v1 import v1_blueprint
from app.services.routing import solve_routing

@v1_blueprint.route('/routes/optimize', methods=['POST'])
def routing():
    try:
        result = request.get_json(force=True)
    except Exception:
        return jsonify({"status": False, "message": "invalid JSON payload"}), 400

    try:
        # TODO: Add background job processing here for scalability
        solution_data = solve_routing(result)
        return jsonify(solution_data), 200
    except Exception as e:
        return jsonify({"status": False, "message": "error processing routing", "error": str(e)}), 500
