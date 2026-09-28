# 🚁 AI-Based Delivery Drone Path Planning System (Streamlit)

Your original Flask 2D prototype, upgraded: type two real addresses and get a collision-free **3D** flight
path over OpenStreetMap buildings that avoids red no-fly cylinders, with a neon 3D ribbon, a 2D overview
inset, drone playback (Orbit / FPV), a dashboard (distance, time, battery, safety score) and an
**AI Explainer Log** that says why the route looks the way it does. No API keys, no JavaScript.

## Run
```bash
pip install -r requirements.txt
streamlit run app.py
```
No internet? Tick **Offline demo city** in the sidebar.

## Deploy from Git
Push this folder to GitHub -> https://share.streamlit.io -> New app -> pick repo, branch, main file `app.py`.
No secrets needed.

## Structure (your layout, extended)
```
app.py                     Streamlit UI (replaces the Flask launcher)
requirements.txt  pytest.ini  .gitignore  .streamlit/config.toml
ai_engine/
  astar.py dijkstra.py route_optimizer.py   your original 2D engine, unchanged
  world.py         NEW  M2 collision model: building height raster, no-fly cylinders, roof-clearance rule
  astar3d.py       NEW  M3 3D A* (x, y, altitude), 26-neighbour moves, string-pull smoothing
  path_planner.py       plan_route (your 2D, unchanged) + plan_route_3d (3D A*, or your 2D A*/Dijkstra
                        flown at a fixed cruise altitude as baselines)
  explainer.py     NEW  M6 "Leg 1: Ascended to 70 m..." text generator
backend/
  physics.py            your 2D distance/time + calculate_flight_metrics (battery, safety score)
  models.py             Point (now with z), DroneMission, FlightPlan
  geo.py           NEW  geocoding (Nominatim), buildings (Overpass), offline demo city
  scene.py         NEW  M1/M4/M5 pydeck: 3D city, red cylinders, neon ribbon, drone, Orbit/FPV, 2D inset
data/default_zones.json default addresses, demo trip and no-fly zone layout (format changed, see below)
tests/                  test_planner_2d.py (yours) + test_planner_3d.py
```

## Milestones (read in this order)
1. Map setup - `backend/scene.py`, `app.py`  2. Collision logic - `ai_engine/world.py`
3. Pathfinding - `ai_engine/astar3d.py` (compare with `astar.py`)  4. Neon path + zones - `scene.py`
5. Animation + camera - `app.py`, `scene.py`  6. Dashboard + explainer - `physics.py`, `explainer.py`

## Tests
`python -m pytest` (offline; covers your original 2D tests plus 3D clearance, zone avoidance, metrics).

## What changed from your version
- `frontend/index.html` and `backend/server.py` (Flask) are gone: Streamlit is now the UI and server.
  Flask is removed from requirements.
- `data/default_zones.json` no longer holds a 2D obstacle grid; it holds app defaults and the zone layout.
- `Point` gained `z`; `DroneMission` gained clearance/cruise settings.

## Known limits
- Flat ground (heights are above ground level; no terrain model). Keep routes 200 m - 4 km.
- The 2D baselines can only go around obstacles, never over them, and may clip building corners diagonally -
  a good comparison against 3D A*, which checks clearance in 3D.
- FPV is a chase camera aimed at the ground ahead of the drone; animation is server-driven (a few fps).
- OSM heights missing -> 10 m. Battery is a made-up model (0.012 %/m + 0.06 %/m climbed).
