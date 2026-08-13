from flask import Flask
from flask_cors import CORS

def create_app():
    app = Flask(__name__)
    CORS(app)
    
    from app.routes import api_blueprint
    app.register_blueprint(api_blueprint)
    
    return app
