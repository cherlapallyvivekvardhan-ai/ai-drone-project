"""Route planning engine: pure Python, no dependencies, no web code."""
import heapq
import math

SIZE = 50          # map is SIZE x SIZE cells
CELL_METRES = 10   # one cell = 10 m
SQRT2 = math.sqrt(2)
MOVES = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

# Default map: a 3x3 block plus a long wall with gaps at both ends.
DEFAULT_OBSTACLES = sorted(
    {(x, y) for x in range(15, 18) for y in range(15, 18)}
    | {(30, y) for y in range(5, 40)}
)


def rectangle(x1, y1, x2, y2):
    """All cells inside a rectangle, in any corner order."""
    xs, ys = sorted((x1, x2)), sorted((y1, y2))
    return {(x, y) for x in range(xs[0], xs[1] + 1) for y in range(ys[0], ys[1] + 1)}


def find_path(start, goal, blocked, algorithm="astar", size=SIZE):
    """Shortest path on an 8-direction grid.

    Dijkstra is A* with no heuristic, so one function does both. They find the
    same shortest distance; A* just searches fewer cells.
    Returns (path, cells_searched). Raises ValueError if no route exists.
    """
    def heuristic(cell):
        if algorithm != "astar":
            return 0
        dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return (dx + dy) + (SQRT2 - 2) * min(dx, dy)

    queue = [(heuristic(start), 0.0, start)]
    cost = {start: 0.0}
    parent = {}
    done = set()

    while queue:
        _, g, cell = heapq.heappop(queue)
        if cell in done:
            continue
        done.add(cell)

        if cell == goal:
            path = [cell]
            while cell in parent:
                cell = parent[cell]
                path.append(cell)
            return path[::-1], len(done)

        x, y = cell
        for dx, dy in MOVES:
            nxt = (x + dx, y + dy)
            if not (0 <= nxt[0] < size and 0 <= nxt[1] < size) or nxt in blocked:
                continue
            # Don't squeeze diagonally between two touching obstacles.
            if dx and dy and ((x + dx, y) in blocked or (x, y + dy) in blocked):
                continue
            new_cost = g + (SQRT2 if dx and dy else 1.0)
            if new_cost < cost.get(nxt, math.inf):
                cost[nxt] = new_cost
                parent[nxt] = cell
                heapq.heappush(queue, (new_cost + heuristic(nxt), new_cost, nxt))

    raise ValueError("No route exists between start and destination.")


def keep_corners(path):
    """Drop points in the middle of straight stretches; keep only the turns."""
    if len(path) < 3:
        return path
    out = [path[0]]
    for prev, cur, nxt in zip(path, path[1:], path[2:]):
        if (cur[0] - prev[0], cur[1] - prev[1]) != (nxt[0] - cur[0], nxt[1] - cur[1]):
            out.append(cur)
    out.append(path[-1])
    return out


def plan(start, goal, blocked, algorithm="astar", speed=10.0, size=SIZE):
    """Validate inputs, plan the route, return a result dict."""
    if algorithm not in ("astar", "dijkstra"):
        raise ValueError("Algorithm must be 'astar' or 'dijkstra'.")
    if speed <= 0:
        raise ValueError("Drone speed must be greater than zero.")
    for name, c in (("start", start), ("destination", goal)):
        if not (0 <= c[0] < size and 0 <= c[1] < size):
            raise ValueError(f"The {name} {c} is outside the {size}x{size} map.")
        if c in blocked:
            raise ValueError(f"The {name} {c} is inside an obstacle.")

    path, searched = find_path(start, goal, blocked, algorithm, size)
    route = keep_corners(path)
    metres = sum(math.dist(a, b) for a, b in zip(route, route[1:])) * CELL_METRES
    return {
        "algorithm": algorithm,
        "route": route,
        "distance_m": round(metres, 1),
        "flight_time": round(metres / speed, 1),
        "waypoints": len(route),
        "cells_searched": searched,
    }
