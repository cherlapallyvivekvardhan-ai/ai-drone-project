import math


def calculate_distance(point1, point2):
    dx = point2["x"] - point1["x"]
    dy = point2["y"] - point1["y"]

    return math.sqrt(dx * dx + dy * dy)


def calculate_route_distance(route):
    total = 0.0

    for i in range(len(route) - 1):
        total += calculate_distance(
            route[i],
            route[i + 1]
        )

    return round(total, 2)


def calculate_route_metrics(route, drone_speed):
    distance = calculate_route_distance(route)

    if drone_speed <= 0:
        raise ValueError("Drone speed must be greater than zero.")

    flight_time = distance / drone_speed

    return {
        "distance": distance,
        "flight_time": round(flight_time, 2)
    }
