import time

from ai_engine.astar import astar
from ai_engine.dijkstra import dijkstra
from ai_engine.grid import build_obstacle_set
from ai_engine.route_optimizer import optimize_route

ALGORITHMS = {"astar": astar, "dijkstra": dijkstra}


def plan_route_detailed(start, destination, obstacles, algorithm="astar",
                        width=50, height=50, smooth=False):
    """Plan a route and return the route plus search statistics."""
    name = str(algorithm).lower()
    if name not in ALGORITHMS:
        raise ValueError("Unsupported algorithm. Use astar or dijkstra.")

    t0 = time.perf_counter()
    raw, stats = ALGORITHMS[name](start, destination, obstacles,
                                  width, height, return_stats=True)
    blocked = build_obstacle_set(obstacles, width, height)
    route = optimize_route(raw, blocked, smooth=smooth)
    elapsed_ms = round((time.perf_counter() - t0) * 1000, 2)

    return {
        "algorithm": name,
        "route": route,
        "raw_route": raw,
        "expanded": stats["expanded"],
        "explored": stats["explored"],
        "compute_ms": elapsed_ms,
    }


def plan_route(start, destination, obstacles, algorithm="astar",
               width=50, height=50, smooth=False):
    """Simple API: returns just the list of waypoints."""
    return plan_route_detailed(start, destination, obstacles, algorithm,
                               width, height, smooth)["route"]
