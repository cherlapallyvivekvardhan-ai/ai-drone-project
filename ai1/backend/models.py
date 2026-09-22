from dataclasses import dataclass, field
from typing import List


@dataclass
class Point:
    x: int
    y: int


@dataclass
class DroneMission:
    start: Point
    destination: Point
    obstacles: List[dict] = field(default_factory=list)
    algorithm: str = "astar"
    drone_speed: float = 10.0
