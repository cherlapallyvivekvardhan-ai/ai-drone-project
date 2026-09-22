"""
Flight-metric estimation for a delivery drone traversing a planned route.

These are simplified, physically-reasonable models suitable for an academic
prototype: constant cruise speed, and battery drain proportional to distance
plus a fixed take-off/landing overhead.
"""

from typing import List
from models.schemas import Coordinate
from services.distance import total_path_distance

# Reference values for a typical small multirotor delivery drone
DRONE_SPEED_KMH: float = 40.0          # Cruise ground speed
BATTERY_RANGE_KM: float = 20.0         # Full-battery range under nominal payload
BATTERY_OVERHEAD_PERCENT: float = 5.0  # Fixed cost for takeoff/landing/hover


def estimate_flight_time_minutes(distance_km: float, speed_kmh: float = DRONE_SPEED_KMH) -> float:
    """Converts a distance and cruise speed into an estimated flight time in minutes."""
    if speed_kmh <= 0:
        return 0.0
    return (distance_km / speed_kmh) * 60.0


def estimate_battery_percent(distance_km: float, battery_range_km: float = BATTERY_RANGE_KM) -> float:
    """
    Estimates the percentage of battery capacity consumed by the flight,
    including a fixed overhead for takeoff, landing, and hover stabilization.
    """
    if battery_range_km <= 0:
        return 100.0

    travel_percent = (distance_km / battery_range_km) * 100.0
    total_percent = travel_percent + BATTERY_OVERHEAD_PERCENT
    return min(total_percent, 100.0)


def compute_route_metrics(route: List[Coordinate]) -> dict:
    """
    Computes the full set of telemetry metrics the frontend displays for a
    planned route: total distance, flight time, cruise speed, waypoint
    count, and estimated battery cost.
    """
    distance_km = total_path_distance(route)
    time_minutes = estimate_flight_time_minutes(distance_km)
    battery_percent = estimate_battery_percent(distance_km)

    return {
        "distance_km": distance_km,
        "estimated_time_minutes": time_minutes,
        "drone_speed_kmh": DRONE_SPEED_KMH,
        "waypoints": len(route),
        "battery_required_percent": battery_percent,
    }
