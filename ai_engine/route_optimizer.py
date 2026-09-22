"""
route_optimizer.py
-------------------
Multi-stop delivery ordering.

Given a depot (start/end point) and a list of delivery points, this
finds a good visiting order (a small Traveling-Salesman-style problem)
using:
  1. Nearest-neighbor construction for a fast initial route.
  2. 2-opt local search to remove crossing/inefficient segments.

Distances between points use the real PathPlanner (A*) so obstacles
and no-fly zones are respected -- this is not straight-line distance.
"""

from __future__ import annotations

from typing import List, Tuple

from ai_engine.path_planner import PathPlanner

Point = Tuple[int, int]


def _pairwise_distances(planner: PathPlanner, points: List[Point]) -> List[List[float]]:
    n = len(points)
    dist = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            planned = planner.plan(points[i], points[j])
            d = planned.distance_km if planned.found else float("inf")
            dist[i][j] = d
            dist[j][i] = d
    return dist


def _nearest_neighbor_order(dist: List[List[float]], start_idx: int) -> List[int]:
    n = len(dist)
    unvisited = set(range(n))
    unvisited.discard(start_idx)
    order = [start_idx]
    current = start_idx

    while unvisited:
        nxt = min(unvisited, key=lambda j: dist[current][j])
        order.append(nxt)
        unvisited.discard(nxt)
        current = nxt

    return order


def _route_length(order: List[int], dist: List[List[float]]) -> float:
    return sum(dist[order[i]][order[i + 1]] for i in range(len(order) - 1))


def _two_opt(order: List[int], dist: List[List[float]]) -> List[int]:
    """Classic 2-opt improvement. Depot (index 0 of `order`) stays fixed."""
    improved = True
    best = order[:]

    while improved:
        improved = False
        for i in range(1, len(best) - 2):
            for j in range(i + 1, len(best) - 1):
                a, b, c, d = best[i - 1], best[i], best[j], best[j + 1]
                delta = (dist[a][c] + dist[b][d]) - (dist[a][b] + dist[c][d])
                if delta < -1e-9:
                    best[i:j + 1] = reversed(best[i:j + 1])
                    improved = True
    return best


def optimize_delivery_order(
    planner: PathPlanner,
    depot: Point,
    deliveries: List[Point],
) -> Tuple[List[Point], float]:
    """
    Returns (ordered_points_including_depot_first, total_distance_km).

    The depot is placed first; the drone is assumed to return to the
    depot at the end automatically by the caller (Route/Simulation),
    so this function returns the *visiting order of the outbound
    trip* starting at the depot.
    """
    if not deliveries:
        return [depot], 0.0

    points = [depot] + deliveries
    dist = _pairwise_distances(planner, points)

    order = _nearest_neighbor_order(dist, start_idx=0)
    order = _two_opt(order, dist)

    ordered_points = [points[i] for i in order]
    total = _route_length(order, dist)
    return ordered_points, total
