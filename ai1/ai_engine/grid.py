"""Shared grid helpers used by every planner."""
import math

# (dx, dy, step cost). Diagonals cost sqrt(2).
DIRECTIONS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def to_cell(point, name="point"):
    """Convert {"x":.., "y":..} to an (int, int) tuple, with clear errors."""
    try:
        return int(round(float(point["x"]))), int(round(float(point["y"])))
    except (KeyError, TypeError, ValueError):
        raise ValueError(f"{name} must look like {{\"x\": 1, \"y\": 2}}.")


def to_point(cell):
    return {"x": cell[0], "y": cell[1]}


def build_obstacle_set(obstacles, width, height):
    cells = set()
    for o in obstacles or []:
        c = to_cell(o, "obstacle")
        if 0 <= c[0] < width and 0 <= c[1] < height:
            cells.add(c)
    return cells


def validate_endpoints(start, goal, obstacles, width, height):
    for label, c in (("Start", start), ("Destination", goal)):
        if not (0 <= c[0] < width and 0 <= c[1] < height):
            raise ValueError(
                f"{label} ({c[0]}, {c[1]}) is outside the {width}x{height} map."
            )
        if c in obstacles:
            raise ValueError(f"{label} ({c[0]}, {c[1]}) is inside an obstacle.")


def neighbors(cell, obstacles, width, height):
    """Yield (neighbor, cost). Diagonal moves may not squeeze past a corner."""
    x, y = cell
    for dx, dy, cost in DIRECTIONS:
        nx, ny = x + dx, y + dy
        if not (0 <= nx < width and 0 <= ny < height):
            continue
        if (nx, ny) in obstacles:
            continue
        if dx and dy and ((x + dx, y) in obstacles or (x, y + dy) in obstacles):
            continue  # no corner cutting
        yield (nx, ny), cost


def octile(a, b):
    """Exact shortest distance on an empty 8-connected grid (admissible)."""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
