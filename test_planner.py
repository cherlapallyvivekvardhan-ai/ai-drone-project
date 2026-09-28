"""Run with:  python test_planner.py   (or pytest)"""
import math

import planner
from planner import DEFAULT_OBSTACLES, find_path, keep_corners, plan, rectangle


def walk(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def raises(fn, *args, **kw):
    try:
        fn(*args, **kw)
    except ValueError:
        return True
    return False


def test_algorithms_agree_and_avoid_obstacles():
    blocked = set(DEFAULT_OBSTACLES)
    a, a_seen = find_path((2, 2), (45, 45), blocked, "astar")
    d, d_seen = find_path((2, 2), (45, 45), blocked, "dijkstra")
    assert a[0] == (2, 2) and a[-1] == (45, 45) and d[-1] == (45, 45)
    assert not (set(a) & blocked) and not (set(d) & blocked)
    assert abs(walk(a) - walk(d)) < 1e-9
    assert a_seen < d_seen


def test_keep_corners():
    assert keep_corners([(0, 0), (1, 0), (2, 0)]) == [(0, 0), (2, 0)]
    assert keep_corners([(0, 0), (1, 0), (2, 0), (2, 1)]) == [(0, 0), (2, 0), (2, 1)]


def test_no_corner_cutting():
    assert raises(find_path, (0, 0), (1, 1), {(1, 0), (0, 1)}, "astar", 2)


def test_metrics():
    r = plan((0, 0), (5, 0), set(), "astar", 10)
    assert r["distance_m"] == 50.0 and r["flight_time"] == 5.0 and r["waypoints"] == 2


def test_bad_input():
    assert raises(plan, (0, 0), (99, 0), set())
    assert raises(plan, (0, 0), (5, 5), set(), "astar", 0)
    assert raises(plan, (0, 0), (5, 5), set(), "bfs")
    assert raises(plan, (5, 5), (9, 9), {(5, 5)})
    wall = {(4, y) for y in range(planner.SIZE)}
    assert raises(plan, (0, 0), (9, 0), wall)


def test_rectangle_any_corner_order():
    assert rectangle(2, 2, 1, 1) == {(1, 1), (1, 2), (2, 1), (2, 2)}


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("All tests passed.")
