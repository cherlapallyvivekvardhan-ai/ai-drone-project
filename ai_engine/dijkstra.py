"""
dijkstra.py
-----------
Dijkstra's algorithm for the drone path-planning engine.

Same grid model as astar.py (8-connected occupancy grid), kept as a
separate, independent algorithm so the planner can compare/benchmark
the two, or fall back to Dijkstra when a custom cost surface makes
the A* heuristic inadmissible.
"""

from __future__ import annotations

import heapq
import math
from typing import Callable, Dict, List, Optional, Tuple

Point = Tuple[int, int]

_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def dijkstra(
    width: int,
    height: int,
    blocked: set,
    start: Point,
    goal: Point,
    extra_cost_fn: Optional[Callable[[Point], float]] = None,
) -> Tuple[Optional[List[Point]], float]:
    """
    Run Dijkstra's algorithm on an implicit grid.

    Signature mirrors astar.astar() so the two are interchangeable
    inside PathPlanner.

    Returns:
        (path, total_cost) or (None, inf) if unreachable.
    """
    if start == goal:
        return [start], 0.0

    def in_bounds(p: Point) -> bool:
        return 0 <= p[0] < width and 0 <= p[1] < height

    if not in_bounds(start) or not in_bounds(goal):
        raise ValueError(f"start {start} or goal {goal} outside grid {width}x{height}")

    dist: Dict[Point, float] = {start: 0.0}
    came_from: Dict[Point, Point] = {}
    visited: set = set()
    heap: List[Tuple[float, Point]] = [(0.0, start)]

    while heap:
        d, current = heapq.heappop(heap)

        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return _reconstruct(came_from, current), d

        for dx, dy, step in _NEIGHBORS:
            neighbor = (current[0] + dx, current[1] + dy)

            if not in_bounds(neighbor) or neighbor in blocked or neighbor in visited:
                continue

            if dx != 0 and dy != 0:
                if (current[0] + dx, current[1]) in blocked and (current[0], current[1] + dy) in blocked:
                    continue

            penalty = extra_cost_fn(neighbor) if extra_cost_fn else 0.0
            if penalty == math.inf:
                continue

            nd = d + step + penalty
            if nd < dist.get(neighbor, math.inf):
                dist[neighbor] = nd
                came_from[neighbor] = current
                heapq.heappush(heap, (nd, neighbor))

    return None, math.inf


def _reconstruct(came_from: Dict[Point, Point], current: Point) -> List[Point]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
