import math
import pytest
from ai_engine.astar3d import plan
from ai_engine.explainer import explain
from ai_engine.path_planner import plan_route_3d
from ai_engine.world import World, make_zones
from backend import geo
from backend.physics import calculate_flight_metrics, sample

GOAL = (900.0, 300.0)


def build(clearance=10.0, seed=7):
    zones = make_zones(GOAL)
    bounds = (-300, -300, 1200, 600)
    b = geo.demo_buildings(bounds, [(0, 0), GOAL], seed=seed)
    return World(bounds, 30, clearance, b, zones, [(0, 0), GOAL])


@pytest.mark.parametrize("algo", ["astar3d", "astar2d", "dijkstra2d"])
def test_all_algorithms_reach_goal_and_avoid_zones(algo):
    w = build()
    res = plan_route_3d(w, (0, 0), GOAL, algo, 90)
    assert res is not None
    p = res.path
    assert p[0][2] == 0 and p[-1][2] == 0
    assert math.dist(p[0][:2], (0, 0)) <= w.cell and math.dist(p[-1][:2], GOAL) <= w.cell
    for x, y, z, *_ in sample(p, 5):
        assert all(math.hypot(x - q.x, y - q.y) >= q.r for q in w.zones)


def test_3d_astar_keeps_roof_clearance_on_many_cities():
    for seed in range(1, 6):
        w = build(seed=seed)
        res = plan(w, (0, 0), GOAL)
        for x, y, z, *_ in sample(res.path, 5):
            assert z < 1 or w.is_pad(x, y) or z >= w.h_at(x, y) + w.clearance - 1e-6


def test_metrics_and_explainer():
    w = build()
    res = plan(w, (0, 0), GOAL)
    m = calculate_flight_metrics(res.path, w, 12)
    assert m["dist"] >= math.hypot(*GOAL) and 0 <= m["score"] <= 100 and m["battery"] > 0
    log = explain(res.path, w, res)
    assert log[0].startswith("Scanned") and any(t.startswith("Leg 1") for t in log)


def test_unknown_algorithm_rejected():
    with pytest.raises(ValueError):
        plan_route_3d(build(), (0, 0), GOAL, "rrt")
