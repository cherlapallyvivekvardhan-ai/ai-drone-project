# 🚁 AI Delivery Drone Path Planner

3D A* drone routing over real Mapbox 3D buildings + terrain, wrapped in Streamlit.
Streamlit hosts the page; map, planner and UI run in the browser.

```
app.py              Streamlit shell (token, inlines web/ files into one iframe)
web/index.html      Layout: 3D map, 2D inset, side panel
web/style.css       Dark neon UI
web/planner.js      Collision grid + 3D A* + metrics + explainer (pure JS, no map)
web/app.js          Mapbox scene, no-fly zones, neon ribbon, animation, UI
```

## Run locally
```bash
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
cp .streamlit/secrets.toml.example .streamlit/secrets.toml   # paste your pk. token
streamlit run app.py
```
Get a free **public** token (starts with `pk.`) at https://account.mapbox.com/ .
It is visible in the browser by design, so add URL restrictions in the Mapbox dashboard.

## Deploy (Streamlit Community Cloud)
1. `git init && git add . && git commit -m "drone planner" && git push` to a GitHub repo
   (`secrets.toml` is git-ignored).
2. https://share.streamlit.io -> New app -> pick repo, branch, `app.py`.
3. App settings -> Secrets -> `MAPBOX_TOKEN = "pk...."` -> Save.

## Learning milestones (read the code in this order)
| # | Milestone | Where |
|---|-----------|-------|
| 1 | Map setup: 3D buildings, terrain, 2D inset | `app.js` top (`map.on('load')`), `index.html` |
| 2 | Collision logic: building height raster, no-fly cylinders, clearance rule | `planner.js` `buildWorld` |
| 3 | Pathfinding: heap, 26-neighbour 3D A*, string-pull smoothing | `planner.js` `Heap`, `plan` |
| 4 | Visible path + no-fly zones: extruded emissive ribbon, red cylinders | `app.js` `ribbon`, `makeZones` |
| 5 | Animation: drone marker, Orbit / FPV free-camera | `app.js` bottom |
| 6 | AI explainer + dashboard metrics | `planner.js` `metrics`, `explain` |

Test the planner without a browser: `node -e "const P=require('./web/planner.js')"`.

## How the planner decides
- World is a grid (>=30 m cells) x 8 altitude levels (0-150 m). A node is flyable only if
  altitude >= roof height + 10 m buffer and it is outside every no-fly zone (+12 m margin).
- Cost = 3D distance x 1.15 when changing altitude x 1.3 when close to roof clearance,
  heuristic = straight 3D distance (admissible), so routes prefer flying around over climbing.
- The raw grid path is smoothed by skipping waypoints whenever the straight segment is collision-free.

## Known limits (good exercises)
- Heights are above ground level (terrain is not part of collision).
- Building data comes from tiles loaded in view; keep routes under ~4 km. Holes in building
  footprints are ignored; missing height defaults to 8 m.
- Battery model is simulated: `0.012 %/m + 0.06 %/m climbed`. Try adding wind, or RRT*.
