import math

# One grid cell is this many metres. Speed is metres per second.
CELL_SIZE_M = 10.0


def calculate_distance(p1, p2):
    return math.hypot(p2["x"] - p1["x"], p2["y"] - p1["y"])


def calculate_route_distance(route):
    """Route length in grid cells."""
    return round(sum(calculate_distance(route[i], route[i + 1])
                     for i in range(len(route) - 1)), 2)


def calculate_route_metrics(route, drone_speed):
    if drone_speed is None or drone_speed <= 0:
        raise ValueError("Drone speed must be greater than zero.")
    cells = calculate_route_distance(route)
    metres = cells * CELL_SIZE_M
    return {
        "distance": cells,
        "distance_m": round(metres, 1),
        "flight_time": round(metres / drone_speed, 2),
    }
