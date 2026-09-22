"""
no_fly_zone.py
--------------
A circular restricted-airspace zone defined in grid-cell coordinates.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class NoFlyZone:
    name: str
    center: Tuple[int, int]
    radius_cells: float

    def contains(self, point: Tuple[int, int]) -> bool:
        dx = point[0] - self.center[0]
        dy = point[1] - self.center[1]
        return math.hypot(dx, dy) <= self.radius_cells

    @classmethod
    def from_dict(cls, data: dict) -> "NoFlyZone":
        return cls(
            name=data["name"],
            center=(data["center"][0], data["center"][1]),
            radius_cells=data["radius_cells"],
        )
