# Drone Mission Platform

An interactive delivery-drone route planner: a Flask API, an A* / Dijkstra
planning engine with obstacle avoidance, and a browser dashboard where you
draw no-fly zones, move the start and destination, and watch the drone fly.

## Run it

Needs Python 3.9+.

- **Windows:** double-click `run.bat`
- **macOS / Linux:** `./run.sh`
- **Manual:** `pip install -r requirements.txt` then `python app.py`

The dashboard opens at http://127.0.0.1:5000. Useful flags:
`--port 8080`, `--no-browser`, `--debug`. If the port is busy it picks the next free one.

## Using the dashboard

1. Pick a map tool: set start, set destination, draw zone, erase zone.
2. Click or drag on the map. The route re-plans automatically.
3. **Plan and fly route** animates the drone. **Compare A\* and Dijkstra** draws both routes and tabulates distance, time, waypoints, cells searched and compute time.
4. **Straighten route** shortens the path with line-of-sight smoothing that never enters a no-fly zone.

## Project layout

```
app.py                  launcher (starts server, opens browser)
ai_engine/              grid.py, astar.py, dijkstra.py, route_optimizer.py, path_planner.py
backend/                server.py (API), physics.py, models.py
frontend/index.html     dashboard (no build step, no external files)
data/default_zones.json city layout with start and destination
tests/test_planner.py   engine and API tests  ->  pytest
```

## API

| Endpoint | Purpose |
|---|---|
| `GET /api/health` | status and available algorithms |
| `GET /api/default-zones` | default map |
| `POST /api/plan-route` | one algorithm; returns route, metrics, searched cells |
| `POST /api/compare` | A* and Dijkstra on the same mission |

Request body: `start`, `destination`, `obstacles` (list of `{x, y}`), `algorithm`,
`drone_speed`, optional `width`, `height`, `smooth`.
Errors return `{"success": false, "error": "..."}` with 400 (bad input) or 422 (no route exists).

## What changed from the first version

- **Fixed:** the route optimizer kept the wrong waypoints; diagonal moves could cut between touching obstacles; the "Dijkstra baseline" used different moves than A*, so results weren't comparable; the server could return 500 for bad input; the frontend path broke depending on the launch folder.
- **Added:** click-to-place start/destination, draw/erase zones, auto re-plan, drone animation, searched-cell overlay, side-by-side comparison, route smoothing, input validation, one-click launchers, real metres (10 m per cell), and 24 tests.

## Next steps

3D altitude, battery limits, GeoJSON/map-image import, moving obstacles.
