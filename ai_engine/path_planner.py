"""
path_planner.py
----------------
High-level path planning interface used by the rest of the app.

Wraps a grid map + a set of static obstacles + a set of no-fly zones,
and exposes a single .plan(start, goal, algorithm) call that:
  1. Picks A* or Dijkstra.
  2. Turns no-fly zones into a soft/hard cost penalty on the grid.
  3. Converts the resulting cell path into real-world distance (km)
     using the map's meters-per-cell scale.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Literal, Optional, Tuple

from ai_engine.astar import astar
from ai_engine.dijkstra import dijkstra
from entities.no_fly_zone import NoFlyZone

Point = Tuple[int, int]
Algorithm = Literal["astar", "dijkstra"]


@dataclass
class PlannedPath:
    algorithm: Algorithm
    waypoints: List[Point]
    cell_cost: float
    distance_km: float
    no_fly_zones_crossed: List[str] = field(default_factory=list)

    @property
    def found(self) -> bool:
        return len(self.waypoints) > 0


class PathPlanner:
    def __init__(
        self,
        width: int,
        height: int,
        blocked_cells: set,
        meters_per_cell: float = 25.0,
        no_fly_zones: Optional[List[NoFlyZone]] = None,
    ) -> None:
        self.width = width
        self.height = height
        self.blocked_cells = blocked_cells
        self.meters_per_cell = meters_per_cell
        self.no_fly_zones = no_fly_zones or []

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    def plan(
        self,
        start: Point,
        goal: Point,
        algorithm: Algorithm = "astar",
        allow_no_fly_soft_cross: bool = False,
    ) -> PlannedPath:
        """
        Plan a path from start to goal.

        If allow_no_fly_soft_cross is False (default), no-fly zones are
        hard-blocked (infinite cost). If True, they're heavily
        penalized but still crossable as a last resort (useful for
        emergency landings).
        """
        cost_fn = self._make_cost_fn(allow_no_fly_soft_cross)
        solver = astar if algorithm == "astar" else dijkstra

        path, cost = solver(self.width, self.height, self.blocked_cells, start, goal, cost_fn)

        if path is None:
            return PlannedPath(algorithm=algorithm, waypoints=[], cell_cost=math.inf, distance_km=math.inf)

        distance_km = self._path_distance_km(path)
        crossed = self._zones_crossed(path)

        return PlannedPath(
            algorithm=algorithm,
            waypoints=path,
            cell_cost=cost,
            distance_km=distance_km,
            no_fly_zones_crossed=crossed,
        )

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _make_cost_fn(self, soft: bool):
        if not self.no_fly_zones:
            return None

        def cost_fn(point: Point) -> float:
            for zone in self.no_fly_zones:
                if zone.contains(point):
                    return 250.0 if soft else math.inf
            return 0.0

        return cost_fn

    def _zones_crossed(self, path: List[Point]) -> List[str]:
        names = []
        for zone in self.no_fly_zones:
            if any(zone.contains(p) for p in path) and zone.name not in names:
                names.append(zone.name)
        return names

    def _path_distance_km(self, path: List[Point]) -> float:
        total_cells = 0.0
        for (x1, y1), (x2, y2) in zip(path, path[1:]):
            total_cells += math.hypot(x2 - x1, y2 - y1)
        return (total_cells * self.meters_per_cell) / 1000.0
