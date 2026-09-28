from dataclasses import dataclass, field
from typing import Any, List


@dataclass
class Point:
    x: float
    y: float
    z: float = 0.0            # altitude above ground (m); 0 for the original 2D use


@dataclass
class DroneMission:
    start: Point
    destination: Point
    obstacles: List[dict] = field(default_factory=list)
    algorithm: str = "astar3d"          # astar3d | astar2d | dijkstra2d
    drone_speed: float = 12.0            # m/s
    clearance: float = 10.0              # roof buffer (m)
    cruise_altitude: float = 50.0        # only used by the 2D baselines


@dataclass
class FlightPlan:
    """Everything the dashboard needs after planning."""
    scene: Any
    metrics: dict
    log: List[str]
    n_buildings: int
    algorithm: str
