# 🚁 AI-Based Delivery Drone Path Planning System

A Streamlit app that plans and flies a delivery-drone mission over a city: pick a START and a DESTINATION on a map,
tune the drone and the planner, and watch an animated 3D flight with a step-by-step explanation of every
distance, every route phase and every re-route or hold.

## Features

- Interactive Folium map: click to set START and DESTINATION (or type coordinates).
- Sidebar controls: cruise altitude and planning ceiling, drone mass and payload, speed, wind vector, battery,
  path-cost weights (distance / climb / risk), obstacle detection range, safety margin, obstacle response
  (auto / prefer re-route / prefer hold), and toggles for real buildings, no-fly zones and moving obstacles.
- Real buildings from OpenStreetMap (Overpass). If the lookup fails, synthetic buildings are used and the app says so.
- 3D occupancy grid, static and moving obstacles, 3D A* planning with line-of-sight smoothing, step-by-step flight
  with dynamic re-routing or holding, energy and flight-time model, built-in validation.
- Metrics, planned and actual path tables, event table, consistency checks.
- Animated 3D viewer (MapLibre GL JS 3.6.2 + deck.gl 8.9.35) with play / pause / replay / seek / speed, a HUD, and
  click-to-inspect. If the libraries cannot load, the viewer shows a friendly message and its information panel and
  playback controls keep working.

## Corrected 3D positioning

Everything uses **one** coordinate reference (see the top of `sim3d.py`):

| Quantity | Meaning |
|---|---|
| `x` | metres **east** of the START point |
| `y` | metres **north** of the START point |
| `z` | metres **above ground level (AGL)**; terrain is assumed flat, so building heights and drone altitude share the same zero |
| lat/lon ↔ x/y | `x = (lon − lon0)·m_per_deg_lon`, `y = (lat − lat0)·m_per_deg_lat` and the exact inverse, using the metres-per-degree values at the START latitude. These two numbers are sent to the viewer (`meta`), so the browser uses the identical mapping |

What this guarantees:

- Start, destination, waypoints, flight frames, buildings, no-fly zones and moving obstacles are all converted with
  the same `GeoFrame`; a drone frame stores both `(x, y, z)` and the `(lat, lon)` computed from them.
- The grid's altitude layers sit at `z = k·dz` with `dz` chosen so the cruise layer is **exactly** the requested
  altitude, and the first/last route points are snapped to the exact start/destination coordinates (not to a grid-cell
  centre), with takeoff and landing as vertical legs at those exact coordinates.
- A frame is recorded at every waypoint, so the flown path *is* the planned polyline (no corner cutting by interpolation).
  The viewer interpolates `lon`, `lat` and `z` linearly in time, which is exact because lon/lat are linear in x/y.
- The drone marker, its vertical drop line (`z → 0`) and its ground shadow (a metre-accurate circle) all use the same
  interpolated `lon/lat`.
- The 2D Folium map and the 3D viewer draw from the same result dictionary, so they always agree.

## Distances: planned vs straight-line vs actual

All distances are in **metres**, computed in the local frame.

- **Planned route distance** – the 3D length of the route the planner chose (climb + cruise + descent + detours).
- **Straight-line distance** – the ground distance START → DESTINATION, ignoring obstacles (both points are at z = 0, so it is
  also the 3D straight-line). Cross-checked against the haversine formula.
- **Actual distance flown** – the 3D distance really flown. It equals planned unless a **re-route** happened; a **hold**
  adds flight time and hover energy but **no** distance.
- **Detour factor** = planned ÷ straight-line.

Each route leg shows its horizontal, vertical and 3D length, and the **cumulative distance from takeoff**
(including the distance flown before any re-route).

## Route-distance and step-by-step explanation features

- **2D map tab:** every leg is clickable (coordinates of both ends, altitude, segment length, cumulative distance,
  closest obstacle and why the leg exists); every waypoint is clickable; re-route and hold events are marked with
  icons whose pop-ups give the location, reason and distance impact; a legend explains the colours; layers can be toggled.
  A **step inspector** slider moves a drone marker and shows position, altitude, distance travelled/remaining, energy
  used/remaining, obstacle clearance and risk, state and the plain-language explanation of that step.
- **3D viewer:** the HUD and side panel update continuously while playing, and identically after pause, replay, seek or a
  speed change, because they are computed only from the playback time. Click a leg, waypoint, event, building, no-fly zone
  or moving obstacle (on the map or in the lists) to inspect it.
- **Phases:** takeoff, climbing, cruising, descending, landing, holding and re-routing each have a short explanation of
  what is happening and why. Arriving at a waypoint states the cumulative distance and what comes next.

## Project structure

```
ai-drone-project/
├── app.py            Streamlit UI (map, sidebar, metrics, tables, step inspector, embeds the viewer)
├── sim3d.py          Simulation engine (standard library only, no Streamlit)
├── viewer.py         Builds the animated 3D viewer as one HTML string
├── requirements.txt  streamlit, folium, streamlit-folium, pandas
└── README.md
```

## Run locally

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
streamlit run app.py
```

Checks (no Streamlit needed):

```bash
python sim3d.py --selftest      # coordinate round-trips, distances, simulations, validation
python viewer.py --selftest     # HTML/JSON embedding + JS interpolation (needs node for the JS part)
```

## Deploy on Streamlit Community Cloud

1. Push this folder to a GitHub repository.
2. Go to <https://share.streamlit.io>, choose **New app**, select the repository and branch, and set the main file to `app.py`.
3. Click **Deploy**. Dependencies come from `requirements.txt`.

The 3D map libraries load from unpkg.com and map tiles from OpenStreetMap / CARTO in the **viewer's browser**, so no
extra server setup is needed. The OpenStreetMap building lookup runs on the Streamlit server; if it is blocked the app
falls back to synthetic buildings and tells you.

## Limitations

- Flat terrain: altitude and building heights are both above ground level; terrain elevation is not modelled.
- Planning is treated as instantaneous (no simulated time passes while a re-route is computed).
- OpenStreetMap building heights are often missing and are then assumed to be 10 m (shown with `~`).
- The local frame is a tangent-plane approximation: accurate to well under 0.5 % for the supported route length (≤ 6 km).
- The viewer's clearance values are interpolated linearly between frames; the table values are exact at each frame.
- The Streamlit map and the 3D viewer are separate widgets, so the 2D step slider and the 3D playback are independent.
