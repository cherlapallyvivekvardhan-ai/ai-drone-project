"""Route clean-up: drop redundant waypoints, then optionally shorten the path."""
import math

CLEARANCE = 0.75  # smoothed legs must stay this far from any obstacle cell


def remove_collinear(route):
    """Keep only the points where the direction of travel changes."""
    if len(route) <= 2:
        return list(route)
    out = [route[0]]
    for i in range(1, len(route) - 1):
        d_in = (route[i]["x"] - route[i - 1]["x"], route[i]["y"] - route[i - 1]["y"])
        d_out = (route[i + 1]["x"] - route[i]["x"], route[i + 1]["y"] - route[i]["y"])
        if d_in != d_out:
            out.append(route[i])
    out.append(route[-1])
    return out


def _dist_point_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def line_is_clear(a, b, blocked):
    """True if the straight leg a->b keeps CLEARANCE from every blocked cell."""
    x0, x1 = sorted((a["x"], b["x"]))
    y0, y1 = sorted((a["y"], b["y"]))
    for (ox, oy) in blocked:
        if x0 - 1 <= ox <= x1 + 1 and y0 - 1 <= oy <= y1 + 1:
            if _dist_point_segment(ox, oy, a["x"], a["y"], b["x"], b["y"]) < CLEARANCE:
                return False
    return True


def smooth_route(route, blocked):
    """Greedy line-of-sight shortcutting: straighter, shorter, never unsafe."""
    if len(route) <= 2:
        return list(route)
    out, i = [route[0]], 0
    while i < len(route) - 1:
        j = len(route) - 1
        while j > i + 1 and not line_is_clear(route[i], route[j], blocked):
            j -= 1
        out.append(route[j])
        i = j
    return out


def optimize_route(route, blocked=None, smooth=False):
    route = remove_collinear(route)
    if smooth and blocked is not None:
        route = smooth_route(route, blocked)
    return route
