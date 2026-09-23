"""
environment.py
Defines the 3D Airspace Environment, static/dynamic obstacles, and environmental forces.
"""

from typing import List, Tuple, Set
import numpy as np


class DynamicObstacle:
    """Represents a moving obstacle in 3D space (e.g., bird, other drone)."""

    def __init__(self, start_pos: Tuple[int, int, int], velocity: Tuple[int, int, int], radius: float = 1.5):
        self.position = np.array(start_pos, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.radius = radius

    def update(self, bounds: Tuple[int, int, int]) -> None:
        """Updates position based on velocity and bounds bouncing."""
        self.position += self.velocity
        for i in range(3):
            if self.position[i] <= 0 or self.position[i] >= bounds[i] - 1:
                self.velocity[i] *= -1  # Reverse direction at boundary


class AirspaceEnvironment:
    """3D Grid space representing the physical operational environment."""

    def __init__(self, bounds: Tuple[int, int, int], wind_vector: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        self.bounds = bounds  # (X, Y, Z) dimensions
        self.wind_vector = np.array(wind_vector, dtype=float)  # (vx, vy, vz) in m/s
        self.static_obstacles: Set[Tuple[int, int, int]] = set()
        self.no_fly_zones: Set[Tuple[int, int, int]] = set()
        self.dynamic_obstacles: List[DynamicObstacle] = []

    def in_bounds(self, pos: Tuple[int, int, int]) -> bool:
        """Check if coordinates are within simulation airspace limits."""
        x, y, z = pos
        return 0 <= x < self.bounds[0] and 0 <= y < self.bounds[1] and 0 <= z < self.bounds[2]

    def add_static_building(self, min_corner: Tuple[int, int, int], max_corner: Tuple[int, int, int]) -> None:
        """Add a cuboid static obstacle representing a building."""
        for x in range(min_corner[0], max_corner[0] + 1):
            for y in range(min_corner[1], max_corner[1] + 1):
                for z in range(min_corner[2], max_corner[2] + 1):
                    if self.in_bounds((x, y, z)):
                        self.static_obstacles.add((x, y, z))

    def add_no_fly_zone(self, center: Tuple[int, int, int], radius: int, height_range: Tuple[int, int]) -> None:
        """Add a cylindrical no-fly zone (restricted airspace)."""
        cx, cy, cz = center
        for x in range(cx - radius, cx + radius + 1):
            for y in range(cy - radius, cy + radius + 1):
                for z in range(height_range[0], height_range[1] + 1):
                    if self.in_bounds((x, y, z)) and ((x - cx) ** 2 + (y - cy) ** 2) <= radius ** 2:
                        self.no_fly_zones.add((x, y, z))

    def is_obstacle(self, pos: Tuple[int, int, int]) -> bool:
        """Returns true if cell contains static obstacle or is inside no-fly zone."""
        return pos in self.static_obstacles or pos in self.no_fly_zones

    def update_dynamic_environment(self) -> None:
        """Advances moving obstacles by 1 time step."""
        for obs in self.dynamic_obstacles:
            obs.update(self.bounds)

    def get_dynamic_repulsion(self, pos: Tuple[int, int, int], influence_radius: float = 3.0) -> float:
        """Calculates distance-based risk penalty from nearby moving objects."""
        pos_np = np.array(pos, dtype=float)
        total_risk = 0.0
        for obs in self.dynamic_obstacles:
            dist = np.linalg.norm(pos_np - obs.position)
            if dist < (obs.radius + influence_radius):
                total_risk += 1.0 / max(dist - obs.radius, 0.1)
        return total_risk
