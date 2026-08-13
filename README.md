# Route Optimization API

This project provides a Route Optimization API powered by **Google OR-Tools** and **Flask**. It solves various routing problems, including Vehicle Routing Problem (VRP), Capacitated VRP (CVRP), and VRP with Time Windows (VRPTW).

It is designed to receive lists of tasks with geographic locations or addresses, geocode them using **Here Maps API**, and use **OSRM** and **Geopy** to calculate the distance/time matrices before feeding them into OR-Tools to compute the most optimal vehicle routes.

## Features
- **Route Optimization**: VRP, CVRP, Time Windows support, and multi-trip configurations.
- **Geocoding & Reverse Geocoding**: Powered by Here Maps to convert raw addresses into valid latitude/longitude coordinates.
- **Distance matrix calculation**: Distance calculations using OSRM and great circle distances.
- **Docker Ready**: Production-ready `Dockerfile` running on Python 3.12 with Gunicorn.

## Prerequisites

- **Python 3.12** or higher.
- **Here Maps API Keys**: The geocoding service requires Here Maps credentials. These should be exposed via a `.env` file.

## Local Setup & Running

1. **Clone the repository**
2. **Set up environment variables:**
   ```bash
   cp .env.example .env
   ```
   Edit `.env` to configure your `OSRM_BASE_URL` (default: http://router.project-osrm.org) and `HERE_API_KEY` for geocoding.
3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Run the Flask application (Development)**:
   ```bash
   flask --app wsgi run --host=127.0.0.1 --port=8000 --debug
   ```
   *Alternatively, if using VS Code, you can launch the debugger using the provided `.vscode/launch.json` configuration.*

3. **Run using Gunicorn (Production)**:
   ```bash
   gunicorn --bind 0.0.0.0:8000 wsgi:app
   ```

## Docker Setup

Build and run the application inside a lightweight container:

```bash
docker build -t route-optimization-api .
docker run -p 8000:8000 -e HERE_API_KEY="your-api-key" route-optimization-api
```

## API Endpoints

- `GET /` : Health check / Info page.
- `POST /api/v1/routes/optimize` : Accepts a JSON payload containing `task_list`, `vehicle_capacities`, `time_limit`, etc., and returns the optimized routes for each vehicle.
  
  **Advanced Optimization Features:**
  - **Service Time**: Pass `"service_time": 600` (in seconds) inside `task_list` items to model unloading/loading duration.
  - **Skill-Based Routing (Tags)**: Pass `"tags": ["East"]` in `task_list` items, and `"vehicle_tags": [["East"], ["West"]]` to restrict tasks to specifically assigned vehicles.
  - **Vehicle Time Windows**: Pass `"vehicle_time_windows": [[0, 28800], [28800, 57600]]` (in seconds from 00:00) to enforce strict working shifts for drivers.
  - **Task Time Windows**: Pass `"time_windows": [[0, 86400], [3600, 7200], ...]` to enforce arrival time constraints on specific tasks.

- `POST /api/v1/estimations/time` : Calculates estimated travel time and distance between two coordinates (`coordinate_origin` and `coordinate_destination`).
- `GET /api/v1/geocode?address=...` : Converts a raw address into geographic coordinates.
- `GET /api/v1/geocode/reverse?coordinate=...` : Converts latitude and longitude coordinates into a human-readable address.

## OSRM (OpenStreetMap Routing Machine) Reference
For calculating exact road distances, the API reaches out to an OSRM backend. 
To run your own local OSRM instance for Indonesia data, you can use the following commands:
```bash
osrm-extract jabodetabek.pbf -p osrm-backend/profiles/car.lua
osrm-contract jabodetabek.osrm
osrm-routed jabodetabek.osrm
```
*(Data source: [OpenStreetMap Indonesia](https://openstreetmap.id/data-openstreetmap-indonesia/))*
