# 🚁 AI Delivery Drone Path Planner (pure Streamlit)

Type two addresses, get a collision-free 3D flight path over real OpenStreetMap buildings, avoiding red
no-fly cylinders - with a neon 3D ribbon, a 2D overview inset, drone playback (Orbit / FPV) and an
"AI explainer" log that says *why* the route looks the way it does. No JavaScript, no API keys.

## Run
```bash
git clone <your-repo-url> && cd drone-planner-streamlit
python -m venv .venv && source .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -r requirements.txt
streamlit run app.py
```
No internet at runtime? Tick **Offline demo city** in the sidebar.

## Deploy (Streamlit Community Cloud)
Push to GitHub -> https://share.streamlit.io -> New app -> repo, branch, `app.py`. No secrets needed.

## Layout / learning milestones
| # | Milestone | File |
|---|-----------|------|
| 1 | Map setup: 3D city, 2D inset (pydeck + CARTO dark basemap) | `drone/scene.py`, `app.py` |
| 2 | Collision logic: building height raster, no-fly cylinders, roof-clearance rule | `drone/planner.py` `World` |
| 3 | Pathfinding: 26-neighbour 3D A* + string-pull smoothing | `drone/planner.py` `plan` |
| 4 | Visible path + zones: glowing 3D PathLayer, translucent red cylinders | `drone/scene.py` |
| 5 | Playback: Fly Drone, progress scrub, Orbit / FPV cameras | `app.py`, `drone/scene.py` |
| 6 | Dashboard + explainer log | `drone/planner.py` `metrics`, `explain` |
| - | Geocoding (Nominatim), buildings (Overpass), offline demo city | `drone/geo.py` |

Tests (no network): `pip install -r requirements-dev.txt && python -m pytest`

## How the planner decides
- Grid cells (>= 30 m) x 8 altitude levels (0-150 m). A node is flyable only if altitude >= roof + buffer
  and it is outside every zone (+12 m margin). Ground level exists only on the two pads.
- Cost = 3D distance, x1.15 when changing altitude, x1.3 near the clearance limit; heuristic = straight 3D
  distance, so the planner prefers going around over climbing.
- Start/end are snapped to the pad-cell centre (up to half a cell, ~15-20 m off the geocoded point).

## Known limits (good exercises)
- Flat ground: heights are above ground level, there is no terrain model. Basemap is 2D tiles with 3D
  objects on top.
- FPV is a chase camera aimed at the ground ahead of the drone (pydeck cannot set camera altitude).
- Animation is server-driven (one map update per frame), so it is a few frames per second, not 60 fps.
- OSM data: missing heights default to 10 m; building relations/holes are ignored. Keep routes < 4 km.
- Battery is a made-up model: 0.012 %/m + 0.06 %/m climbed. Try wind, or swap A* for RRT*.
