import heapq
import itertools

from ai_engine.grid import (build_obstacle_set, neighbors, octile, to_cell,
                            to_point, validate_endpoints)


def astar(start, goal, obstacles, width=50, height=50, return_stats=False):
    """A* on an 8-connected grid. Returns a list of {"x","y"} points.

    With return_stats=True returns (path, stats) where stats holds the number
    of expanded cells and the expanded cells themselves (for visualisation).
    """
    blocked = build_obstacle_set(obstacles, width, height)
    s, g = to_cell(start, "start"), to_cell(goal, "destination")
    validate_endpoints(s, g, blocked, width, height)

    counter = itertools.count()  # tie-breaker so heap never compares tuples of cells
    open_heap = [(octile(s, g), next(counter), s)]
    came_from, g_score, closed = {}, {s: 0.0}, set()
    expanded = []

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expanded.append(current)

        if current == g:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            result = [to_point(c) for c in path]
            if return_stats:
                return result, {"expanded": len(expanded),
                                "explored": [to_point(c) for c in expanded]}
            return result

        for nb, cost in neighbors(current, blocked, width, height):
            tentative = g_score[current] + cost
            if tentative < g_score.get(nb, float("inf")):
                came_from[nb] = current
                g_score[nb] = tentative
                heapq.heappush(open_heap,
                               (tentative + octile(nb, g), next(counter), nb))

    raise ValueError("No valid route exists between start and destination.")
