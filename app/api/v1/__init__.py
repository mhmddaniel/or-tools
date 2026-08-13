from flask import Blueprint

v1_blueprint = Blueprint('api_v1', __name__, url_prefix='/api/v1')

@v1_blueprint.after_request
def after_request(response):
    response.headers.add("Access-Control-Allow-Origin", "*")
    response.headers.add("Access-Control-Allow-Methods", "GET,HEAD,OPTIONS,POST,PUT")
    response.headers.add("Access-Control-Allow-Headers", "*")
    return response

# Import controllers to register their routes with the blueprint
from app.api.v1 import routing, estimations, geocoding
