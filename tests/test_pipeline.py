import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ai_engine.path_planner import PathPlanner
from ai_engine.route_optimizer import optimize_delivery_order
from entities.battery_profile import BatteryProfile
from entities.drone_profile import DroneProfile
from entities.no_fly_zone import NoFlyZone
from main import build_route, load_map, load_no_fly_zones, DEFAULT_MAP, DEFAULT_OBSTACLES
from simulation.simulator import DroneSimulator


def _default_planner() -> PathPlanner:
    width, height, mpc, blocked = load_map(DEFAULT_MAP)
    zones = load_no_fly_zones(DEFAULT_OBSTACLES)
    return PathPlanner(width, height, blocked, mpc, zones)


def test_planner_finds_path_on_sample_map():
    planner = _default_planner()
    result = planner.plan((2, 2), (35, 3))
    assert result.found
    assert result.distance_km > 0


def test_route_optimizer_visits_all_deliveries():
    planner = _default_planner()
    depot = (2, 2)
    deliveries = [(35, 3), (12, 25), (25, 10)]
    order, total_km = optimize_delivery_order(planner, depot, deliveries)
    assert order[0] == depot
    assert set(order) == {depot, *deliveries}
    assert total_km > 0


def test_build_route_returns_to_depot():
    planner = _default_planner()
    depot = (2, 2)
    deliveries = [(35, 3), (12, 25)]
    route = build_route(planner, depot, deliveries, algorithm="astar")
    assert route.stops[0] == depot
    assert route.stops[-1] == depot
    assert route.full_path[0] == depot
    assert route.full_path[-1] == depot


def test_simulation_reports_energy_use():
    planner = _default_planner()
    route = build_route(planner, (2, 2), [(35, 3)], algorithm="astar")
    drone = DroneProfile()
    battery = BatteryProfile(charge_pct=100.0)
    sim = DroneSimulator(drone, battery, payload_kg=1.0)
    result = sim.run(route)
    assert result.total_distance_km == route.distance_km
    assert result.energy_used_wh > 0
    assert 0 <= result.battery_pct_remaining <= 100


def test_low_battery_triggers_failure():
    planner = _default_planner()
    route = build_route(planner, (2, 2), [(35, 3), (12, 25), (25, 10), (3, 27)], algorithm="astar")
    drone = DroneProfile()
    battery = BatteryProfile(charge_pct=5.0)  # almost empty on purpose
    sim = DroneSimulator(drone, battery, payload_kg=2.0)
    result = sim.run(route)
    assert result.success is False
    assert any("Insufficient battery" in w for w in result.warnings)
