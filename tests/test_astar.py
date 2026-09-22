import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ai_engine.astar import astar
from ai_engine.dijkstra import dijkstra


def test_astar_straight_line_no_obstacles():
    path, cost = astar(10, 10, blocked=set(), start=(0, 0), goal=(5, 0))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (5, 0)
    assert math.isclose(cost, 5.0)


def test_astar_routes_around_wall():
    blocked = {(3, y) for y in range(0, 8)}
    path, cost = astar(10, 10, blocked, start=(0, 0), goal=(6, 0))
    assert path is not None
    assert all(p not in blocked for p in path)


def test_astar_unreachable_goal():
    blocked = {(x, 5) for x in range(0, 10)}
    path, cost = astar(10, 10, blocked, start=(0, 0), goal=(0, 9))
    assert path is None
    assert cost == math.inf


def test_dijkstra_matches_astar_cost_no_obstacles():
    _, a_cost = astar(15, 15, set(), (0, 0), (10, 7))
    _, d_cost = dijkstra(15, 15, set(), (0, 0), (10, 7))
    assert math.isclose(a_cost, d_cost, rel_tol=1e-6)


def test_same_start_and_goal():
    path, cost = astar(5, 5, set(), (2, 2), (2, 2))
    assert path == [(2, 2)]
    assert cost == 0.0
