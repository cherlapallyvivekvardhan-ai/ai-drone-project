#!/usr/bin/env python3
"""
main.py
=======
SINGLE ENTRY POINT for the Drone Delivery Path-Planning System.

    Run it with:
        python main.py
        python main.py --algorithm dijkstra --deliveries "12,4 30,25 6,20"
        python main.py --plot --plot-out my_route.png
        python main.py --help

This script wires together every module in the project:

    data/sample_map.json        -> grid + static obstacles
    data/sample_obstacles.json  -> no-fly zones
    ai_engine/path_planner.py   -> A* / Dijkstra route planning
    ai_engine/route_optimizer.py-> multi-stop delivery ordering
    entities/*.py                -> DroneProfile, BatteryProfile, Route, etc.
    simulation/simulator.py     -> flight simulation (time + battery)
    visualization/plotter.py    -> optional PNG map of the route

No external dependencies are required to run the core pipeline.
matplotlib is optional and only used for the --plot image export.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import List, Tuple

from ai_engine.path_planner import PathPlanner
from ai_engine.route_optimizer import optimize_delivery_order
from entities.battery_profile import BatteryProfile
from entities.drone_profile import DroneProfile
from entities.no_fly_zone import NoFlyZone
from entities.route import Route
from entities.user import User
from simulation.simulator import DroneSimulator

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_MAP = BASE_DIR / "data" / "sample_map.json"
DEFAULT_OBSTACLES = BASE_DIR / "data" / "sample_obstacles.json"

Point = Tuple[int, int]

BANNER = r"""
 ____  ____   ___  _   _ _____   ____   _  _____ _   _
|  _ \|  _ \ / _ \| \ | | ____| |  _ \ / \|_   _| | | |
| | | | |_) | | | |  \| |  _|   | |_) / _ \ | | | |_| |
| |_| |  _ <| |_| | |\  | |___  |  __/ ___ \| | |  _  |
|____/|_| \_\\___/|_| \_|_____| |_| /_/   \_\_| |_| |_|

     AI-Powered Delivery Drone Path Planning System
"""


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #
def load_map(path: Path) -> Tuple[int, int, float, set]:
    data = json.loads(path.read_text())
    width = data["width"]
    height = data["height"]
    meters_per_cell = data.get("meters_per_cell", 25.0)
    blocked = {tuple(cell) for cell in data.get("obstacles", [])}
    return width, height, meters_per_cell, blocked


def load_no_fly_zones(path: Path) -> List[NoFlyZone]:
    data = json.loads(path.read_text())
    return [NoFlyZone.from_dict(z) for z in data.get("no_fly_zones", [])]


def parse_points(raw: str) -> List[Point]:
    """Parse '12,4 30,25 6,20' -> [(12, 4), (30, 25), (6, 20)]."""
    points = []
    for chunk in raw.split():
        x_str, y_str = chunk.split(",")
        points.append((int(x_str), int(y_str)))
    return points


# --------------------------------------------------------------------------- #
# Core pipeline
# --------------------------------------------------------------------------- #
def build_route(
    planner: PathPlanner,
    depot: Point,
    deliveries: List[Point],
    algorithm: str,
) -> Route:
    """Optimize delivery order, then stitch A*/Dijkstra sub-paths + return to depot."""
    ordered_stops, _ = optimize_delivery_order(planner, depot, deliveries)
    ordered_stops.append(depot)  # return to base

    full_path: List[Point] = []
    total_distance = 0.0
    zones_crossed: List[str] = []

    for a, b in zip(ordered_stops, ordered_stops[1:]):
        planned = planner.plan(a, b, algorithm=algorithm)
        if not planned.found:
            raise RuntimeError(f"No path found between {a} and {b} -- check obstacles/no-fly zones.")

        # Avoid duplicating the shared waypoint between consecutive legs.
        segment = planned.waypoints if not full_path else planned.waypoints[1:]
        full_path.extend(segment)
        total_distance += planned.distance_km
        for z in planned.no_fly_zones_crossed:
            if z not in zones_crossed:
                zones_crossed.append(z)

    return Route(
        stops=ordered_stops,
        full_path=full_path,
        distance_km=total_distance,
        algorithm=algorithm,
        no_fly_zones_crossed=zones_crossed,
    )


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="AI-powered delivery drone path planning system.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--map", type=Path, default=DEFAULT_MAP, help="Path to the grid map JSON file")
    p.add_argument("--obstacles", type=Path, default=DEFAULT_OBSTACLES, help="Path to the no-fly zones JSON file")
    p.add_argument("--depot", type=str, default="2,2", help="Depot / start point as 'x,y'")
    p.add_argument(
        "--deliveries", type=str, default="35,3 12,25 25,10 3,27",
        help="Space-separated delivery points as 'x,y x,y ...'",
    )
    p.add_argument("--algorithm", choices=["astar", "dijkstra"], default="astar", help="Pathfinding algorithm")
    p.add_argument("--payload-kg", type=float, default=1.5, help="Payload weight carried on the mission")
    p.add_argument("--battery-charge-pct", type=float, default=100.0, help="Starting battery charge percentage")
    p.add_argument("--plot", action="store_true", help="Export a PNG visualization of the route (needs matplotlib)")
    p.add_argument("--plot-out", type=str, default="route_map.png", help="Output path for the PNG plot")
    p.add_argument("--quiet", action="store_true", help="Suppress the banner and verbose step logging")
    return p


def main(argv: List[str] = None) -> int:
    args = build_arg_parser().parse_args(argv)

    if not args.quiet:
        print(BANNER)

    user = User(username="operator", role="pilot")
    if not args.quiet:
        print(f"Operator: {user}")
        print(f"Loading map from:      {args.map}")
        print(f"Loading no-fly zones:  {args.obstacles}\n")

    width, height, meters_per_cell, blocked = load_map(args.map)
    no_fly_zones = load_no_fly_zones(args.obstacles)

    depot = parse_points(args.depot)[0]
    deliveries = parse_points(args.deliveries)

    planner = PathPlanner(
        width=width,
        height=height,
        blocked_cells=blocked,
        meters_per_cell=meters_per_cell,
        no_fly_zones=no_fly_zones,
    )

    drone = DroneProfile()
    battery = BatteryProfile(charge_pct=args.battery_charge_pct)

    if not args.quiet:
        print(f"Drone:    {drone.name}  (max speed {drone.max_speed_kmh} km/h, payload {args.payload_kg} kg)")
        print(f"Battery:  {battery.name}  ({battery.capacity_wh} Wh, starting at {battery.charge_pct:.0f}%)")
        print(f"Depot:              {depot}")
        print(f"Delivery stops:     {deliveries}")
        print(f"Algorithm:          {args.algorithm}")
        print("\nPlanning route ...")

    t0 = time.perf_counter()
    route = build_route(planner, depot, deliveries, args.algorithm)
    plan_time_ms = (time.perf_counter() - t0) * 1000

    print(f"\n{route.summary()}")
    print(f"Route planned in {plan_time_ms:.1f} ms across {len(route.full_path)} grid cells.")

    simulator = DroneSimulator(drone=drone, battery=battery, payload_kg=args.payload_kg)
    result = simulator.run(route)
    result.print_report()

    if args.plot:
        from visualization.plotter import plot_route
        out = plot_route(
            width=width,
            height=height,
            blocked_cells=blocked,
            no_fly_zones=no_fly_zones,
            route_path=route.full_path,
            stops=route.stops,
            output_path=args.plot_out,
        )
        if out:
            print(f"\nRoute map saved to: {out}")

    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
