import math
from models.schemas import Coordinate

EARTH_RADIUS_KM = 6371.0088


def haversine_distance(coord1: Coordinate, coord2: Coordinate) -> float:
    """
    Computes great-circle distance between two geographic coordinates using the
    Haversine formula.

    Args:
        coord1: Starting coordinate (latitude/longitude in decimal degrees).
        coord2: Ending coordinate (latitude/longitude in decimal degrees).

    Returns:
        Distance between the two points in kilometers.
    """
    lat1_rad = math.radians(coord1.latitude)
    lat2_rad = math.radians(coord2.latitude)
    delta_lat = math.radians(coord2.latitude - coord1.latitude)
    delta_lon = math.radians(coord2.longitude - coord1.longitude)

    a = (
        math.sin(delta_lat / 2) ** 2
        + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return EARTH_RADIUS_KM * c


def total_path_distance(route: list[Coordinate]) -> float:
    """
    Sums the haversine distance across every consecutive pair of waypoints
    in a multi-point route.

    Args:
        route: Ordered list of coordinates representing the flight path.

    Returns:
        Total distance in kilometers across all route segments.
    """
    if len(route) < 2:
        return 0.0

    total = 0.0
    for i in range(len(route) - 1):
        total += haversine_distance(route[i], route[i + 1])
    return total
