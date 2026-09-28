import math
import numpy as np
from drone import geo, planner as P


def build(clearance=10.0, seed=7):
    goal = (900.0, 300.0)
    zones = P.make_zones(goal)
    bounds = (-300, -300, 1200, 600)
    b = geo.demo_buildings(bounds, [(0, 0), goal], seed=seed)
    w = P.World(bounds, 30, clearance, b, zones, [(0, 0), goal])
    return w, goal, P.plan(w, (0, 0), goal)


def test_path_found_and_endpoints_exact():
    w, goal, res = build()
    assert res is not None
    assert res.path[0][2] == 0 and res.path[-1][2] == 0
    assert math.dist(res.path[0][:2], (0, 0)) <= w.cell and math.dist(res.path[-1][:2], goal) <= w.cell


def test_path_respects_zones_and_roof_clearance():
    w, _, res = build()
    for x, y, z, *_ in P.sample(res.path, 5):
        if z < 1:
            continue                                   # pads
        assert all(math.hypot(x - q.x, y - q.y) >= q.r for q in w.zones)
        assert z >= w.h_at(x, y) + w.clearance - 1e-6 or w.is_pad(x, y)


def test_metrics_and_explainer():
    w, _, res = build()
    m = P.metrics(res.path, w)
    assert m["dist"] >= math.hypot(900, 300) and 0 <= m["score"] <= 100
    log = P.explain(res.path, w, res)
    assert log[0].startswith("Scanned") and any("Leg 1" in t for t in log)
