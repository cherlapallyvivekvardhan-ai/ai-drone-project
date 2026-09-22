# AI-Based Delivery Drone Mission Platform

## What this project contains

- FastAPI backend API.
- Browser frontend with an interactive canvas map.
- JSON / GeoJSON zone upload.
- 3D occupancy-grid representation.
- 26-neighbour A* path finding with Euclidean heuristic.
- Dijkstra baseline comparison.
- Collision / no-fly-zone rasterization.
- Path smoothing into fewer flight waypoints.
- Battery, wind and kinematics estimates.
- Multi-objective route selection.
- Swagger API documentation at `/docs`.

## Important coordinate model

This academic prototype uses a **local Cartesian coordinate system in metres**:
- X = east/west-like local axis
- Y = north/south-like local axis
- Z = altitude in metres

The GeoJSON adapter currently treats GeoJSON coordinates as local X/Y values. It does **not** convert latitude/longitude to metres. For real geographic drone deployment, add a verified projection/geodesy layer and a validated aviation/geofence data source.

## Setup

Python 3.10+ recommended.

Windows:
```powershell
py -m venv .venv
.venv\Scripts\activate
python -m pip install -r requirements.txt
python app.py
```

Linux/macOS:
```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python app.py
```

Then open `http://127.0.0.1:8000`.

## API flow

1. Browser sends start, destination, zones, grid, drone and wind parameters to `POST /api/plan`.
2. Backend rasterizes zones into occupied 3D cells.
3. Start and destination are validated.
4. A* searches for a route.
5. Dijkstra can run as a baseline.
6. Grid path is smoothed with collision checks.
7. Physics module estimates time, energy and battery reserve.
8. Optimizer selects the lowest composite score among available candidates.
9. Frontend renders the resulting waypoints and mission metrics.

## Example API request

```json
{
  "start": {"x":30,"y":40,"z":30},
  "destination": {"x":360,"y":330,"z":40},
  "zones": [],
  "grid": {"size_x":41,"size_y":36,"size_z":9,"cell_size_m":10},
  "drone": {"cruise_speed_mps":12,"battery_wh":250},
  "wind": {"x":2,"y":-1,"z":0},
  "algorithm": "astar",
  "optimize": true
}
```

## Limitations

This is a research/academic planning simulator, not an autonomous flight-control or certified aviation system. It does not command a real drone, connect to flight controllers, or guarantee legal/physical flight safety. Real deployment requires authoritative geofencing, obstacle data, terrain/elevation data, aircraft-specific performance models, communications, fail-safe behavior, and applicable aviation approvals.
