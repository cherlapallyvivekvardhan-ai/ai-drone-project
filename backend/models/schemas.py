from pydantic import BaseModel, Field
from typing import List


class Coordinate(BaseModel):
    latitude: float = Field(..., ge=-90.0, le=90.0, description="Latitude in decimal degrees")
    longitude: float = Field(..., ge=-180.0, le=180.0, description="Longitude in decimal degrees")


class RouteRequest(BaseModel):
    start: Coordinate
    destination: Coordinate


class RouteResponse(BaseModel):
    success: bool
    route: List[Coordinate]
    distance_km: float
    estimated_time_minutes: float
    drone_speed_kmh: float
    waypoints: int
    battery_required_percent: float
    message: str
