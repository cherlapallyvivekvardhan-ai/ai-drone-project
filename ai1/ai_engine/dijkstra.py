import heapq
import itertools

from ai_engine.grid import (build_obstacle_set, neighbors, to_cell, to_point,
                            validate_endpoints)


def dijkstra(start, goal, obstacles, width=50, height=50, return_stats=False):
    """Uniform-cost search with the same moves as A*, so the comparison is fair.

    It finds the same shortest distance but searches many more cells, because
    it has no heuristic pointing it toward the goal.
    """
    blocked = build_obstacle_set(obstacles, width, height)
    s, g = to_cell(start, "start"), to_cell(goal, "destination")
    validate_endpoints(s, g, blocked, width, height)

    counter = itertools.count()
    queue = [(0.0, next(counter), s)]
    dist, previous, done = {s: 0.0}, {}, set()
    expanded = []

    while queue:
        d, _, current = heapq.heappop(queue)
        if current in done:
            continue
        done.add(current)
        expanded.append(current)

        if current == g:
            path = [current]
            while current in previous:
                current = previous[current]
                path.append(current)
            path.reverse()
            result = [to_point(c) for c in path]
            if return_stats:
                return result, {"expanded": len(expanded),
                                "explored": [to_point(c) for c in expanded]}
            return result

        for nb, cost in neighbors(current, blocked, width, height):
            nd = d + cost
            if nd < dist.get(nb, float("inf")):
                dist[nb] = nd
                previous[nb] = current
                heapq.heappush(queue, (nd, next(counter), nb))

    raise ValueError("No valid route exists between start and destination.")
