import math

import numpy as np


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

# ---------------------------------------------------------------------------
# 3D additions: used by the Streamlit dashboard (distance, time, battery, safety)
# ---------------------------------------------------------------------------
def bearing(a, b) -> float:
    """Compass bearing (deg, 0 = north) from a to b, both (x, y, ...)."""
    return (math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) + 360) % 360


def sample(path, step):
    """Resample every `step` m -> (n, 5) array [x, y, z, heading_x, heading_y] for ribbons/animation."""
    out, hx, hy = [], *(path[-1][:2] - path[0][:2])
    for a, b in zip(path[:-1], path[1:]):
        if math.hypot(*(b[:2] - a[:2])) > 1:
            hx, hy = b[:2] - a[:2]
        d = math.hypot(hx, hy) or 1
        n = max(1, math.ceil(np.linalg.norm(b - a) / step))
        out += [[*(a + (b - a) * t / n), hx / d, hy / d] for t in range(n)]
    out.append([*path[-1], out[-1][3], out[-1][4]])
    return np.array(out)


def calculate_flight_metrics(path, w, speed=12.0) -> dict:
    """Milestone 6a - dashboard numbers (simulated battery: 0.012 %/m + 0.06 %/m climbed)."""
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    climb = np.clip(np.diff(path[:, 2]), 0, None).sum()
    pts = sample(path, 10)
    vert = [p[2] - w.h_at(p[0], p[1]) for p in pts if p[2] >= 25 and not w.is_pad(p[0], p[1])] or [99]
    nfz = [math.hypot(p[0] - z.x, p[1] - z.y) - z.r for p in pts for z in w.zones] or [999]
    margin = min((min(vert) - w.clearance) / 20, (min(nfz) - 12) / 40)        # 0 = legal minimum, 1 = generous
    score = round(max(0, min(100, 60 + 40 * margin)))
    return dict(dist=float(seg.sum()), seconds=float(seg.sum() / speed), battery=float(seg.sum() * .012 + climb * .06),
                min_vert=float(min(vert)), min_nfz=float(min(nfz)), score=score)
