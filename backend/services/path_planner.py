"""
A* pathfinding over a discretized geographic grid.

The grid currently has no static obstacles baked in (delivery drones fly in
open airspace), but the cell-based no-fly-zone list makes it straightforward
to extend with restricted zones, altitude bands, or weather cells later.
"""

import heapq
import math
from typing import List, Optional, Set, Tuple

from models.schemas import Coordinate
from utils.grid import GridConverter

Cell = Tuple[int, int]

# 8-directional movement (N, S, E, W + diagonals)
NEIGHBOR_OFFSETS = [
    (-1, 0), (1, 0), (0, -1), (0, 1),
    (-1, -1), (-1, 1), (1, -1), (1, 1),
]


def _euclidean(a: Cell, b: Cell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _neighbors(cell: Cell, resolution: int, no_fly_zones: Set[Cell]) -> List[Cell]:
    row, col = cell
    result = []
    for d_row, d_col in NEIGHBOR_OFFSETS:
        n_row, n_col = row + d_row, col + d_col
        if 0 <= n_row < resolution and 0 <= n_col < resolution:
            if (n_row, n_col) not in no_fly_zones:
                result.append((n_row, n_col))
    return result


def _a_star(start: Cell, goal: Cell, resolution: int, no_fly_zones: Set[Cell]) -> Optional[List[Cell]]:
    """Standard A* search with an admissible Euclidean-distance heuristic."""
    open_heap: List[Tuple[float, Cell]] = [(0.0, start)]
    came_from: dict[Cell, Cell] = {}
    g_score = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for neighbor in _neighbors(current, resolution, no_fly_zones):
            step_cost = _euclidean(current, neighbor)
            tentative_g = g_score[current] + step_cost

            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + _euclidean(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return None  # No path found


def plan_path(
    start: Coordinate,
    destination: Coordinate,
    resolution: int = 25,
    no_fly_zones: Optional[Set[Cell]] = None,
) -> List[Coordinate]:
    """
    Runs A* over a grid built between the start and destination coordinates
    and returns the resulting flight path as a list of geographic waypoints.

    Falls back to a direct two-point route if no grid path is found (e.g.
    if the no-fly zones fully block the corridor).
    """
    no_fly_zones = no_fly_zones or set()
    converter = GridConverter.from_endpoints(start, destination, resolution=resolution)

    start_cell = converter.to_cell(start)
    goal_cell = converter.to_cell(destination)

    cell_path = _a_star(start_cell, goal_cell, resolution, no_fly_zones)

    if not cell_path:
        return [start, destination]

    waypoints = [converter.to_coordinate(cell) for cell in cell_path]

    # Snap the first/last waypoints exactly to the requested endpoints so the
    # rendered route always starts/ends precisely where the user picked.
    waypoints[0] = start
    waypoints[-1] = destination

    return waypoints
