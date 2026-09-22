# Drone Delivery Path Planning System (Pure Python)

An AI-based delivery-drone path-planning system, rebuilt as a single,
self-contained **pure Python** project — no frontend framework, no
web server required. Everything runs from one command.

## ▶️ MAIN EXECUTION FILE

```
main.py
```

Full path once unzipped:

```
drone_path_planner/main.py
```

Run it with:

```bash
cd drone_path_planner
python main.py
```

That's it — no build step, no `npm install`. The core pipeline uses
**only the Python standard library**.

## What it does

Running `main.py` will:

1. Load a grid map (`data/sample_map.json`) with static obstacles.
2. Load no-fly zones (`data/sample_obstacles.json`).
3. Optimize the visiting order of delivery stops (`ai_engine/route_optimizer.py`).
4. Plan the full route with A* or Dijkstra (`ai_engine/path_planner.py`,
   `ai_engine/astar.py`, `ai_engine/dijkstra.py`), routing around obstacles
   and restricted airspace.
5. Simulate the flight (`simulation/simulator.py`), tracking flight time,
   energy use, and battery state of charge, and flag any battery or
   airspace warnings.
6. Print a full mission report to the console.
7. Optionally export a PNG map of the route (`visualization/plotter.py`).

## Usage

```bash
# Default mission (sample map, 4 deliveries, A*)
python main.py

# Use Dijkstra instead of A*
python main.py --algorithm dijkstra

# Custom depot + delivery points ("x,y x,y ...")
python main.py --depot 0,0 --deliveries "10,10 20,5 30,20"

# Heavier payload, partially depleted battery
python main.py --payload-kg 2.5 --battery-charge-pct 60

# Export a visual map of the route (requires: pip install matplotlib)
python main.py --plot --plot-out my_mission.png

# See all options
python main.py --help
```

## Project layout

```
drone_path_planner/
├── main.py                     <-- RUN THIS
├── requirements.txt             (matplotlib, optional, only for --plot)
├── ai_engine/
│   ├── astar.py                 A* grid pathfinding
│   ├── dijkstra.py               Dijkstra grid pathfinding
│   ├── path_planner.py          Combines grid + no-fly zones + algorithm choice
│   └── route_optimizer.py       Multi-stop delivery ordering (NN + 2-opt)
├── entities/
│   ├── drone_profile.py         Drone speed/power/payload specs
│   ├── battery_profile.py       Battery capacity/reserve/state of charge
│   ├── no_fly_zone.py           Circular restricted-airspace zones
│   ├── route.py                 A planned, stitched multi-stop route
│   ├── simulation.py            Simulation result + event timeline
│   └── user.py                  Minimal operator record
├── simulation/
│   └── simulator.py              Steps the drone along a route (time + battery)
├── visualization/
│   └── plotter.py                Optional matplotlib PNG export
├── data/
│   ├── sample_map.json           40x30 grid with static obstacles
│   └── sample_obstacles.json     3 sample no-fly zones
└── tests/
    ├── test_astar.py             Unit tests for A* / Dijkstra
    └── test_pipeline.py          End-to-end pipeline tests
```

## Running the tests

```bash
pip install pytest
python -m pytest tests/ -v
```

## Using your own map

Edit or replace `data/sample_map.json`:

```json
{
  "width": 40,
  "height": 30,
  "meters_per_cell": 25,
  "obstacles": [[10, 5], [10, 6]]
}
```

And `data/sample_obstacles.json` for no-fly zones:

```json
{
  "no_fly_zones": [
    { "name": "Airport", "center": [25, 22], "radius_cells": 4 }
  ]
}
```

Then point `main.py` at them with `--map` / `--obstacles`.
