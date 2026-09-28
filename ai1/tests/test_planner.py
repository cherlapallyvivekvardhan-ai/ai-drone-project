import json
import math
from pathlib import Path

import pytest

from ai_engine.grid import build_obstacle_set
from ai_engine.path_planner import plan_route, plan_route_detailed
from ai_engine.route_optimizer import line_is_clear, remove_collinear
from backend.physics import calculate_route_metrics
from backend.server import app

P = lambda x, y: {"x": x, "y": y}
ZONES = json.loads((Path(__file__).parent.parent / "data" / "default_zones.json").read_text())


def length(route):
    return sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(route, route[1:]))


@pytest.mark.parametrize("algo", ["astar", "dijkstra"])
def test_route_endpoints(algo):
    route = plan_route(P(2, 2), P(10, 10), [P(5, 5)], algo)
    assert route[0] == P(2, 2) and route[-1] == P(10, 10)


def test_route_avoids_obstacles():
    obs = ZONES["obstacles"]
    blocked = build_obstacle_set(obs, 50, 50)
    for algo in ("astar", "dijkstra"):
        r = plan_route_detailed(ZONES["start"], ZONES["destination"], obs, algo)["raw_route"]
        assert all((p["x"], p["y"]) not in blocked for p in r)


def test_astar_matches_dijkstra_cost_but_searches_less():
    obs = ZONES["obstacles"]
    a = plan_route_detailed(ZONES["start"], ZONES["destination"], obs, "astar")
    d = plan_route_detailed(ZONES["start"], ZONES["destination"], obs, "dijkstra")
    assert length(a["raw_route"]) == pytest.approx(length(d["raw_route"]))
    assert a["expanded"] < d["expanded"]


def test_no_corner_cutting():
    # Diagonal gap between two touching obstacles must not be used.
    obs = [P(1, 0), P(0, 1)]
    with pytest.raises(ValueError):
        plan_route(P(0, 0), P(1, 1), obs, "astar", width=2, height=2)


def test_no_route_raises():
    wall = [P(5, y) for y in range(10)]
    with pytest.raises(ValueError, match="No valid route"):
        plan_route(P(0, 5), P(9, 5), wall, "astar", width=10, height=10)


def test_start_inside_obstacle_rejected():
    with pytest.raises(ValueError, match="inside an obstacle"):
        plan_route(P(3, 3), P(9, 9), [P(3, 3)], "dijkstra")


def test_out_of_bounds_rejected():
    with pytest.raises(ValueError, match="outside"):
        plan_route(P(3, 3), P(99, 9), [], "astar")


def test_unknown_algorithm():
    with pytest.raises(ValueError, match="Unsupported"):
        plan_route(P(0, 0), P(3, 3), [], "bfs")


def test_remove_collinear_keeps_corners():
    r = [P(0, 0), P(1, 0), P(2, 0), P(2, 1), P(2, 2)]
    assert remove_collinear(r) == [P(0, 0), P(2, 0), P(2, 2)]


def test_straight_line_collapses_to_two_points():
    assert len(plan_route(P(0, 0), P(9, 0), [], "astar")) == 2


def test_smoothing_is_never_longer_and_stays_clear():
    obs = ZONES["obstacles"]
    blocked = build_obstacle_set(obs, 50, 50)
    plain = plan_route(ZONES["start"], ZONES["destination"], obs, "astar")
    smooth = plan_route(ZONES["start"], ZONES["destination"], obs, "astar", smooth=True)
    assert length(smooth) <= length(plain) + 1e-9
    assert all(line_is_clear(a, b, blocked) for a, b in zip(smooth, smooth[1:]))


def test_metrics():
    m = calculate_route_metrics([P(0, 0), P(3, 4)], 10)
    assert m["distance"] == 5.0 and m["flight_time"] == 5.0
    with pytest.raises(ValueError):
        calculate_route_metrics([P(0, 0), P(1, 1)], 0)


# ---- HTTP API ----------------------------------------------------------
@pytest.fixture
def client():
    return app.test_client()


def body(**kw):
    d = {"start": P(2, 2), "destination": P(20, 20), "obstacles": [], "algorithm": "astar"}
    d.update(kw)
    return d


def test_home_page(client):
    assert b"Drone" in client.get("/").data


def test_default_zones(client):
    assert client.get("/api/default-zones").get_json()["width"] == 50


def test_plan_route_ok(client):
    r = client.post("/api/plan-route", json=body())
    j = r.get_json()
    assert r.status_code == 200 and j["success"] and j["waypoints"] == 2


def test_compare_endpoint(client):
    j = client.post("/api/compare", json=body(obstacles=ZONES["obstacles"],
                    start=ZONES["start"], destination=ZONES["destination"])).get_json()
    assert set(j["results"]) == {"astar", "dijkstra"}


@pytest.mark.parametrize("payload,status", [
    ({}, 400),
    (body(drone_speed=0), 400),
    (body(drone_speed="fast"), 400),
    (body(algorithm="nope"), 400),
    (body(destination=P(500, 5)), 400),
    (body(obstacles=[P(20, 20)]), 400),
])
def test_bad_requests_are_400(client, payload, status):
    r = client.post("/api/plan-route", json=payload)
    assert r.status_code == status and r.get_json()["success"] is False


def test_unreachable_is_422(client):
    wall = [P(10, y) for y in range(50)]
    r = client.post("/api/plan-route", json=body(obstacles=wall))
    assert r.status_code == 422
