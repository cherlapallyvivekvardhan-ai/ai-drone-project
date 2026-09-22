#!/usr/bin/env python3
"""
================================================================================
 AI DELIVERY DRONE MISSION PLATFORM  --  FULLY EXECUTED DEMO
================================================================================

WHAT THIS FILE IS
------------------
A single, self-contained, ZERO-EXTRA-DEPENDENCY Python script that runs the
FULL mission-planning pipeline end-to-end, in your terminal, with clear
colourful step-by-step output -- so you can SEE every module connect and
SEE it actually execute, without needing to install FastAPI/uvicorn or open
a browser.

It uses the exact same "brain" as the web app (the ai_engine/ and
backend/physics.py modules) -- it just skips the web server + pydantic
layer, so it runs with only the Python standard library.

HOW TO RUN
----------
    python demo_run.py

That's it. No pip install needed for this file specifically.
(The full web app in app.py still needs `pip install -r requirements.txt`.)

PIPELINE THIS SCRIPT EXECUTES (same as PROJECT_GUIDE.md)
----------------------------------------------------------
  1. Load mission zones (no-fly / obstacle volumes)            -> data/default_zones.json
  2. Rasterize zones into a 3D occupancy grid                  -> ai_engine/path_planner.py
  3. Validate start & destination are not inside a zone        -> ai_engine/path_planner.py
  4. Search a collision-free route with A*                     -> ai_engine/astar.py
  5. Search a collision-free route with Dijkstra (baseline)    -> ai_engine/dijkstra.py
  6. Smooth each raw grid path into fewer flight waypoints      -> ai_engine/path_planner.py
  7. Compute distance / time / energy / battery for each route -> backend/physics.py
  8. Pick the best route with a multi-objective score           -> ai_engine/route_optimizer.py
  9. Print a colourful mission report to the terminal
================================================================================
"""

import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

# ---------------------------------------------------------------------------
# Make sure this script can find the project's own packages (ai_engine,
# backend) no matter which directory it is launched from.
# ---------------------------------------------------------------------------
BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

from ai_engine.astar import astar
from ai_engine.dijkstra import dijkstra
from ai_engine.path_planner import rasterize_zones, normalize_grid_point, smooth_path
from ai_engine.route_optimizer import choose_route
from backend.physics import route_metrics


# ===========================================================================
# 1. TINY COLOUR ENGINE  (pure stdlib -- no colorama / rich required)
# ===========================================================================
class Colour:
    """Minimal ANSI colour helper. Auto-disables itself on terminals that
    don't support colour (e.g. some CI logs / Windows legacy consoles)."""

    _ENABLED = sys.stdout.isatty()

    RESET = "\033[0m" if _ENABLED else ""
    BOLD = "\033[1m" if _ENABLED else ""
    DIM = "\033[2m" if _ENABLED else ""

    RED = "\033[91m" if _ENABLED else ""
    GREEN = "\033[92m" if _ENABLED else ""
    YELLOW = "\033[93m" if _ENABLED else ""
    BLUE = "\033[94m" if _ENABLED else ""
    MAGENTA = "\033[95m" if _ENABLED else ""
    CYAN = "\033[96m" if _ENABLED else ""
    WHITE = "\033[97m" if _ENABLED else ""
    GREY = "\033[90m" if _ENABLED else ""

    BG_BLUE = "\033[44m" if _ENABLED else ""
    BG_GREEN = "\033[42m" if _ENABLED else ""
    BG_MAGENTA = "\033[45m" if _ENABLED else ""

    @classmethod
    def wrap(cls, text, *codes):
        return f"{''.join(codes)}{text}{cls.RESET}"


C = Colour


def banner():
    line = "=" * 78
    print(C.wrap(line, C.CYAN, C.BOLD))
    print(C.wrap("  AI-BASED DELIVERY DRONE PATH-PLANNING SYSTEM  --  LIVE EXECUTION", C.CYAN, C.BOLD))
    print(C.wrap(line, C.CYAN, C.BOLD))
    print()


def step(number, title):
    print(C.wrap(f"\n[STEP {number}] ", C.BG_BLUE + C.WHITE, C.BOLD) + " " + C.wrap(title, C.BLUE, C.BOLD))
    print(C.wrap("-" * 78, C.GREY))


def ok(msg):
    print(C.wrap("  [OK]  ", C.GREEN, C.BOLD) + msg)


def info(msg):
    print(C.wrap("  [..]  ", C.CYAN) + msg)


def warn(msg):
    print(C.wrap("  [!!]  ", C.YELLOW, C.BOLD) + msg)


def fail(msg):
    print(C.wrap("  [XX]  ", C.RED, C.BOLD) + msg)


def kv(key, value, colour=C.WHITE):
    print(f"      {C.wrap(key + ':', C.GREY):<28} {C.wrap(value, colour, C.BOLD)}")


def spinner_pause(seconds=0.15):
    """Tiny artificial pause so the step-by-step execution is readable
    instead of flashing past instantly. Purely cosmetic."""
    time.sleep(seconds)


# ===========================================================================
# 2. LIGHTWEIGHT CONFIG OBJECTS
#    (Same shape/fields as backend/models.py, but plain objects so this
#     demo needs zero third-party packages.)
# ===========================================================================
def build_default_config():
    grid = SimpleNamespace(size_x=41, size_y=36, size_z=9, cell_size_m=10.0)

    drone = SimpleNamespace(
        cruise_speed_mps=12.0,
        battery_wh=250.0,
        base_power_w=180.0,
        climb_power_w_per_mps=35.0,
        wind_sensitivity=0.35,
        safety_margin=1.25,
    )

    wind = SimpleNamespace(x=2.0, y=-1.0, z=0.0)

    start = {"x": 30, "y": 40, "z": 30}
    destination = {"x": 360, "y": 330, "z": 40}

    return grid, drone, wind, start, destination


# ===========================================================================
# 3. THE FULL MISSION PIPELINE
# ===========================================================================
def run_mission():
    banner()

    # ---- STEP 1: Load zones ------------------------------------------------
    step(1, "Loading no-fly / obstacle zones")
    zones_path = BASE_DIR / "data" / "default_zones.json"
    zones = json.loads(zones_path.read_text(encoding="utf-8"))["zones"]
    spinner_pause()
    ok(f"Loaded {len(zones)} zone(s) from {zones_path.name}")
    for z in zones:
        colour = C.RED if z["type"] == "no-fly" else C.YELLOW
        kv(f"  - {z['id']}", f"{z['name']}  [{z['type']}]", colour)

    # ---- STEP 2: Build config ------------------------------------------------
    step(2, "Building grid / drone / wind configuration")
    grid, drone, wind, start, destination = build_default_config()
    spinner_pause()
    ok("Configuration ready")
    kv("Grid size (cells)", f"{grid.size_x} x {grid.size_y} x {grid.size_z}")
    kv("Cell size", f"{grid.cell_size_m} m")
    kv("Drone cruise speed", f"{drone.cruise_speed_mps} m/s")
    kv("Drone battery", f"{drone.battery_wh} Wh")
    kv("Wind vector", f"({wind.x}, {wind.y}, {wind.z}) m/s")
    kv("Start (world m)", f"{start}")
    kv("Destination (world m)", f"{destination}")

    # ---- STEP 3: Rasterize obstacles ---------------------------------------
    step(3, "Rasterizing zones into the 3D occupancy grid")
    margin = grid.cell_size_m * 0.25
    t0 = time.time()
    occupied = rasterize_zones(zones, grid, margin)
    elapsed = time.time() - t0
    ok(f"Rasterized {len(occupied)} occupied grid cell(s) in {elapsed:.3f}s")

    # ---- STEP 4: Validate start / goal --------------------------------------
    step(4, "Validating start & destination")
    start_cell = normalize_grid_point(start, grid)
    goal_cell = normalize_grid_point(destination, grid)
    if start_cell in occupied:
        fail("Start point is inside a restricted zone. Aborting mission.")
        sys.exit(1)
    if goal_cell in occupied:
        fail("Destination point is inside a restricted zone. Aborting mission.")
        sys.exit(1)
    ok("Start and destination are clear of restricted airspace")
    kv("Start grid cell", str(start_cell))
    kv("Destination grid cell", str(goal_cell))

    # ---- STEP 5: A* search --------------------------------------------------
    step(5, "Running A* search (26-neighbour, 3D)")
    size = (grid.size_x, grid.size_y, grid.size_z)
    t0 = time.time()
    path_a, expanded_a = astar(occupied, start_cell, goal_cell, size)
    t_astar = time.time() - t0
    candidates = []
    if path_a:
        ok(f"A* found a path -- {len(path_a)} raw grid nodes, {expanded_a} nodes expanded, {t_astar*1000:.1f} ms")
        smooth_a = smooth_path(path_a, occupied, grid)
        m_a = route_metrics(smooth_a, drone, wind)
        candidates.append({"algorithm": "A*", "path": smooth_a, "metrics": m_a, "expanded_nodes": expanded_a})
    else:
        warn("A* could not find a collision-free path.")

    # ---- STEP 6: Dijkstra baseline -----------------------------------------
    step(6, "Running Dijkstra search (baseline comparison)")
    t0 = time.time()
    path_d, expanded_d = dijkstra(occupied, start_cell, goal_cell, size)
    t_dij = time.time() - t0
    if path_d:
        ok(f"Dijkstra found a path -- {len(path_d)} raw grid nodes, {expanded_d} nodes expanded, {t_dij*1000:.1f} ms")
        smooth_d = smooth_path(path_d, occupied, grid)
        m_d = route_metrics(smooth_d, drone, wind)
        candidates.append({"algorithm": "Dijkstra", "path": smooth_d, "metrics": m_d, "expanded_nodes": expanded_d})
    else:
        warn("Dijkstra could not find a collision-free path.")

    if not candidates:
        fail("No collision-free route exists in the configured 3D grid. Mission aborted.")
        sys.exit(1)

    # ---- STEP 7: Physics / metrics ------------------------------------------
    step(7, "Computing flight physics (distance / time / energy / battery)")
    for c in candidates:
        m = c["metrics"]
        battery_colour = C.GREEN if m["battery_feasible"] else C.RED
        print(C.wrap(f"\n  >> {c['algorithm']}", C.MAGENTA, C.BOLD))
        kv("Waypoints", str(len(c["path"])))
        kv("Distance", f"{m['distance_m']} m")
        kv("Flight time", f"{m['time_s']} s")
        kv("Energy required", f"{m['energy_wh']} Wh")
        kv("Battery remaining", f"{m['battery_remaining_wh']} Wh", battery_colour)
        kv("Battery feasible", str(m["battery_feasible"]), battery_colour)
        kv("Nodes expanded", str(c["expanded_nodes"]))

    # ---- STEP 8: Multi-objective route selection ----------------------------
    step(8, "Selecting the best route (multi-objective optimizer)")
    selected = choose_route(candidates)
    ok(f"Selected algorithm: {selected['algorithm']}  (composite score: {selected.get('score', 'n/a')})")

    # ---- STEP 9: Final mission report ---------------------------------------
    step(9, "FINAL MISSION REPORT")
    print()
    print(C.wrap("  " + "#" * 74, C.GREEN, C.BOLD))
    print(C.wrap(f"  #  SELECTED ROUTE : {selected['algorithm']}", C.GREEN, C.BOLD))
    print(C.wrap("  " + "#" * 74, C.GREEN, C.BOLD))
    print()
    m = selected["metrics"]
    kv("Total distance", f"{m['distance_m']} m", C.CYAN)
    kv("Estimated flight time", f"{m['time_s']} s  (~{m['time_s']/60:.1f} min)", C.CYAN)
    kv("Estimated energy use", f"{m['energy_wh']} Wh", C.CYAN)
    kv("Battery remaining", f"{m['battery_remaining_wh']} Wh", C.GREEN if m["battery_feasible"] else C.RED)
    kv("Wind adjustment factor", f"{m['wind_factor']}", C.CYAN)
    print()
    print(C.wrap("  Waypoints (world metres):", C.YELLOW, C.BOLD))
    for i, wp in enumerate(selected["path"]):
        marker = "START" if i == 0 else ("END" if i == len(selected["path"]) - 1 else f"WP{i}")
        print(f"      {C.wrap(marker, C.MAGENTA, C.BOLD):<14} "
              f"x={wp['x']:>8.2f}  y={wp['y']:>8.2f}  z={wp['z']:>7.2f}")

    print()
    print(C.wrap("=" * 78, C.CYAN, C.BOLD))
    print(C.wrap("  MISSION PLANNING COMPLETE -- all modules executed and connected OK", C.GREEN, C.BOLD))
    print(C.wrap("=" * 78, C.CYAN, C.BOLD))
    print()
    print(C.wrap("  Tip: run the full interactive web dashboard with:", C.GREY))
    print(C.wrap("       pip install -r requirements.txt && python app.py", C.GREY, C.BOLD))
    print(C.wrap("       then open http://127.0.0.1:8000", C.GREY, C.BOLD))
    print()

    return selected


if __name__ == "__main__":
    run_mission()
