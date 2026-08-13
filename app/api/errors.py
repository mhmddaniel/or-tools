from flask import jsonify
from werkzeug.exceptions import HTTPException
from app.api import api_blueprint

@api_blueprint.app_errorhandler(HTTPException)
def handle_http_exception(e):
    return jsonify({
        "status": False,
        "message": e.description,
        "error": e.name
    }), e.code

@api_blueprint.app_errorhandler(Exception)
def handle_generic_exception(e):
    return jsonify({
        "status": False,
        "message": "Internal server error",
        "error": str(e)
    }), 500
