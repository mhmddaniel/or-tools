import requests
from geopy import distance

def get_distance_osrm(coordinate_origin: str, coordinate_destination: str) -> str:
    url = f"https://osrm.mile.app/route/v1/driving/{coordinate_origin};{coordinate_destination}"
    headers = {}
    response = requests.get(url, headers=headers)
    return response.text

def compute_distance_matrix(locations: list, slack: int) -> dict:
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                dist = distance.great_circle(from_node, to_node).km
                distances[from_counter][to_counter] = int(round(dist * 1000) + slack)
    return distances

def compute_ovrp_distance_matrix(locations: list, slack: int) -> dict:
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == 0 or to_counter == 0:
                distances[from_counter][to_counter] = 0
            elif from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                dist = distance.great_circle(from_node, to_node).km
                distances[from_counter][to_counter] = int(round(dist * 1000) + slack)
    return distances
