import os
import requests
from geopy import distance

def get_distance_osrm(coordinate_origin: str, coordinate_destination: str) -> str:
    base_url = os.environ.get("OSRM_BASE_URL", "http://router.project-osrm.org")
    url = f"{base_url}/route/v1/driving/{coordinate_origin};{coordinate_destination}"
    headers = {}
    response = requests.get(url, headers=headers)
    return response.text

def get_osrm_matrix(locations: list) -> dict:
    """Fetches the N x N distance and duration matrices from OSRM table API. Returns None if it fails."""
    if not locations:
        return None
        
    try:
        base_url = os.environ.get("OSRM_BASE_URL", "http://router.project-osrm.org")
        coords_str = ";".join([f"{lon},{lat}" for lat, lon in locations])
        url = f"{base_url}/table/v1/driving/{coords_str}?annotations=distance,duration"
        response = requests.get(url, timeout=10)
        
        if response.status_code == 200:
            data = response.json()
            if "distances" in data and "durations" in data:
                return {"distances": data["distances"], "durations": data["durations"]}
    except Exception as e:
        print(f"OSRM table failed: {e}")
    return None

def compute_distance_matrix(locations: list, slack: int) -> dict:
    """Returns (distance_matrix, time_matrix)"""
    distances = {}
    times = {}
    osrm_data = get_osrm_matrix(locations)
    
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        times[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
                times[from_counter][to_counter] = 0
            else:
                if osrm_data and from_counter < len(osrm_data["distances"]) and to_counter < len(osrm_data["distances"][from_counter]):
                    dist_meters = osrm_data["distances"][from_counter][to_counter]
                    dur_seconds = osrm_data["durations"][from_counter][to_counter]
                    distances[from_counter][to_counter] = int(round(dist_meters)) + slack
                    times[from_counter][to_counter] = int(round(dur_seconds))
                else:
                    dist = distance.great_circle(from_node, to_node).km
                    # Fallback speed 30km/h => 8.33 m/s
                    dist_meters = dist * 1000
                    distances[from_counter][to_counter] = int(round(dist_meters)) + slack
                    times[from_counter][to_counter] = int(round(dist_meters / 8.33))
                    
    return distances, times

def compute_ovrp_distance_matrix(locations: list, slack: int) -> dict:
    distances = {}
    times = {}
    osrm_data = get_osrm_matrix(locations)
    
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        times[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == 0 or to_counter == 0:
                distances[from_counter][to_counter] = 0
                times[from_counter][to_counter] = 0
            elif from_counter == to_counter:
                distances[from_counter][to_counter] = 0
                times[from_counter][to_counter] = 0
            else:
                if osrm_data and from_counter < len(osrm_data["distances"]) and to_counter < len(osrm_data["distances"][from_counter]):
                    dist_meters = osrm_data["distances"][from_counter][to_counter]
                    dur_seconds = osrm_data["durations"][from_counter][to_counter]
                    distances[from_counter][to_counter] = int(round(dist_meters)) + slack
                    times[from_counter][to_counter] = int(round(dur_seconds))
                else:
                    dist = distance.great_circle(from_node, to_node).km
                    dist_meters = dist * 1000
                    distances[from_counter][to_counter] = int(round(dist_meters)) + slack
                    times[from_counter][to_counter] = int(round(dist_meters / 8.33))
                    
    return distances, times
