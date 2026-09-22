"""
astar.py
--------
A* grid pathfinding for the drone path-planning engine.

The grid is a simple 2D occupancy grid: cells are either free (0) or
blocked (1) by a static obstacle. A* finds the shortest 8-connected
path between two cells, using Euclidean distance as both the step
cost and the heuristic (admissible + consistent).
"""

from __future__ import annotations

import heapq
import math
from typing import Callable, Dict, List, Optional, Tuple

Point = Tuple[int, int]

# 8-connected movement: (dx, dy, step_cost)
_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def _heuristic(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(
    width: int,
    height: int,
    blocked: set,
    start: Point,
    goal: Point,
    extra_cost_fn: Optional[Callable[[Point], float]] = None,
) -> Tuple[Optional[List[Point]], float]:
    """
    Run A* on an implicit grid.

    Args:
        width, height: grid dimensions.
        blocked: set of (x, y) cells that are impassable (static obstacles).
        start, goal: (x, y) grid cells.
        extra_cost_fn: optional callable(point) -> extra cost added when
            entering that cell. Used by the path planner to make no-fly
            zones "expensive but not impossible", or to fold in wind/
            terrain penalties. Return math.inf to hard-block a cell.

    Returns:
        (path, total_cost) where path is a list of (x, y) points from
        start to goal inclusive, or (None, inf) if no path exists.
    """
    if start == goal:
        return [start], 0.0

    def in_bounds(p: Point) -> bool:
        return 0 <= p[0] < width and 0 <= p[1] < height

    if not in_bounds(start) or not in_bounds(goal):
        raise ValueError(f"start {start} or goal {goal} outside grid {width}x{height}")

    open_heap: List[Tuple[float, Point]] = [(0.0, start)]
    came_from: Dict[Point, Point] = {}
    g_score: Dict[Point, float] = {start: 0.0}
    closed: set = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        if current == goal:
            return _reconstruct(came_from, current), g_score[current]

        closed.add(current)

        for dx, dy, step in _NEIGHBORS:
            neighbor = (current[0] + dx, current[1] + dy)

            if not in_bounds(neighbor) or neighbor in blocked or neighbor in closed:
                continue

            # Disallow cutting diagonally between two blocked corners.
            if dx != 0 and dy != 0:
                if (current[0] + dx, current[1]) in blocked and (current[0], current[1] + dy) in blocked:
                    continue

            penalty = extra_cost_fn(neighbor) if extra_cost_fn else 0.0
            if penalty == math.inf:
                continue

            tentative_g = g_score[current] + step + penalty

            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + _heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return None, math.inf


def _reconstruct(came_from: Dict[Point, Point], current: Point) -> List[Point]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
