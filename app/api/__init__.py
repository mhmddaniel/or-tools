from flask import Blueprint, jsonify

api_blueprint = Blueprint('api', __name__)

@api_blueprint.after_request
def after_request(response):
    response.headers.add("Access-Control-Allow-Origin", "*")
    response.headers.add("Access-Control-Allow-Methods", "GET,HEAD,OPTIONS,POST,PUT")
    response.headers.add("Access-Control-Allow-Headers", "*")
    return response

@api_blueprint.route('/', methods=['GET'])
def home():
    return "<h1>Automatic Routing</h1><p>This engine is an API for automatic routing.</p>"

from app.api import errors
