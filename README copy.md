route-optimization

<!-- run routing engine api -->
cd /routing
. venv/bin/activate
python3 wsgi.py

<!-- on update -->
```
service routing stop
service routing start
```
<!-- run open street map routing service -->
```
osrm-extract jabodetabek.pbf -p osrm-backend/profiles/car.lua
osrm-contract jabodetabek.osrm
osrm-routed jabodetabek.osrm
```
<!-- osm data source -->
https://openstreetmap.id/data-openstreetmap-indonesia/