from ai_engine.astar import astar
from ai_engine.dijkstra import dijkstra
from ai_engine.route_optimizer import optimize_route
from ai_engine.astar3d import PlanResult, plan as plan_3d, smooth

import time
import numpy as np


# ---------------------------------------------------------------------------
# Original 2D planner (unchanged) - still used by tests/test_planner_2d.py
# ---------------------------------------------------------------------------
def plan_route(
    start,
    destination,
    obstacles,
    algorithm="astar"
):

    if algorithm.lower() == "astar":

        route = astar(
            start,
            destination,
            obstacles
        )

    elif algorithm.lower() == "dijkstra":

        route = dijkstra(
            start,
            destination,
            obstacles
        )

    else:
        raise ValueError(
            "Unsupported algorithm. "
            "Use astar or dijkstra."
        )

    route = optimize_route(route)

    return route


# ---------------------------------------------------------------------------
# 3D planner: one entry point for the three algorithms the dashboard offers
# ---------------------------------------------------------------------------
ALGORITHMS = {
    "astar3d": "3D A* (climb over or steer around)",
    "astar2d": "A* 2D grid at fixed cruise altitude",
    "dijkstra2d": "Dijkstra 2D grid at fixed cruise altitude",
}


def plan_route_3d(world, start, goal, algorithm="astar3d", cruise_altitude=50.0):
    """Return a PlanResult (path = (n, 3) array of x, y, altitude) or None.

    * astar3d    - search x, y and altitude together (ai_engine/astar3d.py).
    * astar2d / dijkstra2d - reuse the ORIGINAL 2D engine on the cells that are blocked at one
      cruise altitude, then climb from the pad, cruise, and descend onto the destination pad.
      Good baselines: they can only go around obstacles, never over them.
    """
    if algorithm == "astar3d":
        return plan_3d(world, start, goal)
    if algorithm not in ("astar2d", "dijkstra2d"):
        raise ValueError(f"Unsupported algorithm {algorithm!r}. Use {', '.join(ALGORITHMS)}.")
    t0 = time.time()
    k = min(range(1, len(world.alts)), key=lambda i: abs(world.alts[i] - cruise_altitude))
    solver = astar if algorithm == "astar2d" else dijkstra
    s, g = {"x": world.ci(start[0]), "y": world.cj(start[1])}, {"x": world.ci(goal[0]), "y": world.cj(goal[1])}
    try:
        cells = optimize_route(solver(s, g, world.blocked_cells(k), world.nx, world.ny))
    except ValueError:
        return None
    pts = [(world.minx + (c["x"] + .5) * world.cell, world.miny + (c["y"] + .5) * world.cell, float(world.alts[k])) for c in cells]
    raw = [(pts[0][0], pts[0][1], 0.0)] + pts + [(pts[-1][0], pts[-1][1], 0.0)]
    return PlanResult(smooth(world, raw), len(cells), len(cells), int((time.time() - t0) * 1000))
