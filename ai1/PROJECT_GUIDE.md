# Full Project Execution Guide

## Architecture

Browser UI → FastAPI `/api/plan` → Pydantic validation → 3D zone rasterization → A* / Dijkstra → path smoothing → physics model → multi-objective optimizer → JSON result → colorful map dashboard.

## Modules

- `app.py`: application entry point, CORS, static frontend, upload and planning routes.
- `ai_engine/astar.py`: 26-neighbour 3D A*.
- `ai_engine/dijkstra.py`: Dijkstra baseline.
- `ai_engine/path_planner.py`: converts world coordinates to grid cells, rasterizes restricted volumes and smooths paths.
- `ai_engine/route_optimizer.py`: combines distance, time and energy into a transparent route score.
- `backend/models.py`: typed request models.
- `backend/physics.py`: simplified wind, time and energy calculations.
- `backend/server.py`: orchestration layer.
- `data/default_zones.json`: sample obstacles and no-fly volumes.
- `frontend/index.html`: colorful responsive dashboard and interactive map.

## Execution

```bash
python -m venv .venv
# Windows:
.venv\Scripts\activate
# Linux/macOS:
source .venv/bin/activate

python -m pip install -r requirements.txt
python app.py
```

Open `http://127.0.0.1:8000`.

## API

- `GET /api/health`
- `GET /api/zones`
- `POST /api/upload-map`
- `POST /api/plan`
- `GET /docs`

## Algorithm pipeline

1. Receive mission.
2. Validate coordinates and parameters.
3. Convert the local map to a 3D occupancy grid.
4. Mark obstacle/no-fly cells as occupied.
5. Search with A* and/or Dijkstra.
6. Convert cells back to metres.
7. Remove unnecessary intermediate waypoints only when line-of-sight remains collision-free.
8. Calculate distance, estimated time, wind factor and energy.
9. Reject/penalize battery-infeasible routes.
10. Return the selected route and comparison data to the UI.

## Research/deployment boundary

The project is a self-contained academic planning simulator. A real drone system would require authoritative aviation geofencing, terrain/elevation, obstacle sensing, aircraft-specific performance models, communications, fail-safe logic and regulatory approval. This package intentionally does not control a real aircraft.
