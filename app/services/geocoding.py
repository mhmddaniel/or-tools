import os
import geopy
from geopy.geocoders import Here
from app.utils.helpers import clean_address

def get_here_client():
    api_key = os.environ.get("HERE_API_KEY")
    app_id = os.environ.get("HERE_APP_ID")
    app_code = os.environ.get("HERE_APP_CODE")
    
    if api_key:
        return Here(api_key=api_key)
    else:
        return Here(app_id=app_id, app_code=app_code)

def create_location_model(data: dict) -> list:
    task_list = data.get('task_list', [])
    geopy.geocoders.options.default_timeout = 15
    location = []
    
    geolocator = get_here_client()
    
    for index, task in enumerate(task_list):
        t = (0, 0)
        if "lat" in task and "lon" in task:
            t = (task["lat"], task["lon"])
            task["address"] = index
        else:
            try:
                address = clean_address(task.get("address", ""))
                glocation = geolocator.geocode(address)
                if glocation and hasattr(glocation, 'address'):
                    t = (glocation.latitude, glocation.longitude)
            except Exception:
                t = (0, 0)
        location.append(t)
    return location

def geocode_address(address_str: str) -> str:
    geolocator = get_here_client()
    address = clean_address(address_str)
    try:
        glocation = geolocator.geocode(address)
        if glocation and hasattr(glocation, 'address'):
            return f"{glocation.latitude},{glocation.longitude}"
    except Exception:
        pass
    return "0,0"

def reverse_geocode(coordinate: str) -> str:
    geolocator = get_here_client()
    try:
        glocation = geolocator.reverse(query=coordinate)
        if glocation and hasattr(glocation, 'address'):
            return glocation.address
    except Exception:
        pass
    return ""
