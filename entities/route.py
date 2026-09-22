"""
route.py
--------
A planned delivery route: the full stitched path plus summary stats.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

Point = Tuple[int, int]


@dataclass
class Route:
    stops: List[Point]                # ordered depot -> delivery1 -> ... -> depot
    full_path: List[Point]            # every grid cell visited, stitched together
    distance_km: float
    algorithm: str
    no_fly_zones_crossed: List[str] = field(default_factory=list)

    def summary(self) -> str:
        zones = ", ".join(self.no_fly_zones_crossed) if self.no_fly_zones_crossed else "none"
        return (
            f"Route[{self.algorithm}]: {len(self.stops) - 1} stop(s), "
            f"{self.distance_km:.2f} km, no-fly zones crossed: {zones}"
        )
