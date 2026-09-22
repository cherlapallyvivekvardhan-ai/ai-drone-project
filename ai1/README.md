# AI-Based Delivery Drone Path Planning System

A fully connected prototype: an interactive web dashboard, a Flask backend,
and an AI path-planning engine (A* and Dijkstra) with obstacle avoidance,
route optimization, and flight-metric calculation.

## Project structure

```
drone_mission_platform/
├── app.py                    # Main application launcher
├── requirements.txt          # Python dependencies
├── ai_engine/
│   ├── astar.py               # A* pathfinding
│   ├── dijkstra.py            # Dijkstra comparison
│   ├── path_planner.py        # Main AI planner/controller
│   └── route_optimizer.py     # Route optimization
├── backend/
│   ├── server.py               # API + frontend connection
│   ├── models.py                # Mission/map data models
│   └── physics.py                # Distance, speed, flight time
├── data/
│   └── default_zones.json     # Default map/obstacle data
├── frontend/
│   └── index.html              # Interactive UI
└── tests/
    └── test_planner.py        # AI/path-planning tests
```

## Setup

1. Make sure you have Python 3.9+ installed.
2. From the project root, install dependencies:

```bash
pip install -r requirements.txt
```

## Run the application

```bash
python app.py
```

Then open your browser to:

```
http://127.0.0.1:5000
```

You'll see the mission dashboard. Click **Load Map Obstacles**, set a
start/destination and speed, choose an algorithm, and click
**Plan Drone Route** to see the AI-generated path, distance, flight time,
and waypoint count.

## Run tests

```bash
pytest tests/
```

## How it works

```
USER
 -> frontend/index.html
 -> (HTTP/JSON) backend/server.py
 -> backend/models.py, backend/physics.py
 -> ai_engine/path_planner.py
      -> astar.py / dijkstra.py -> route_optimizer.py
 -> Optimized Drone Route
 -> back through backend/server.py -> frontend/index.html
 -> Map + Start + End + Obstacles + Route + Distance + Time
```

## Limitation & next steps

This is a 2D grid prototype, not yet a true 3D drone simulator or real
geographic map system. Suggested next layer: map upload (image/GeoJSON),
clickable start/end points, 3D altitude (`x, y, z`), no-fly zones, drone
battery constraints, and animated drone movement — all on top of this same
architecture.
