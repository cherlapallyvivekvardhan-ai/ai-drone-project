"""AI-Based Delivery Drone Path Planning System - Streamlit app.   Run:  streamlit run app.py"""
import json, math, time
from pathlib import Path

import numpy as np
import streamlit as st

from ai_engine.explainer import explain
from ai_engine.path_planner import ALGORITHMS, plan_route_3d
from ai_engine.world import World, make_zones
from backend import geo
from backend.models import FlightPlan
from backend.physics import calculate_flight_metrics
from backend.scene import Scene

CFG = json.loads((Path(__file__).parent / "data" / "default_zones.json").read_text())

st.set_page_config(page_title="AI Drone Mission Platform", page_icon="🚁", layout="wide")
st.title("🚁 AI-Based Delivery Drone Path Planning System")
st.caption("AI mission planning · 3D obstacle avoidance · no-fly zones · route explanation")


@st.cache_data(show_spinner=False, ttl=3600)
def cached_buildings(origin, bounds):
    return geo.fetch_buildings(origin, bounds)


def run_planning(start, dest, demo, algorithm, cruise, clearance, n_zones, speed) -> FlightPlan:
    """Geocode -> zones -> buildings -> collision world -> planner -> metrics -> explainer."""
    if demo:
        origin, goal = tuple(CFG["demo"]["origin"]), tuple(float(v) for v in CFG["demo"]["goal_m"])
    else:
        origin = geo.geocode(start)
        goal = tuple(float(v) for v in geo.to_local(origin, *geo.geocode(dest)))
    if not 200 <= math.hypot(*goal) <= 4000:
        raise ValueError(f"Points are {math.hypot(*goal):.0f} m apart - choose 200 m to 4 km.")
    zones = make_zones(goal, n_zones, CFG["zone_layout"])
    xs = [0, goal[0]] + [z.x + s * z.r for z in zones for s in (-1, 1)]
    ys = [0, goal[1]] + [z.y + s * z.r for z in zones for s in (-1, 1)]
    bounds = (min(xs) - 250, min(ys) - 250, max(xs) + 250, max(ys) + 250)
    buildings = geo.demo_buildings(bounds, [(0, 0), goal]) if demo else cached_buildings(origin, bounds)
    cell = max(30.0, max(bounds[2] - bounds[0], bounds[3] - bounds[1]) / 90)
    world = World(bounds, cell, clearance, buildings, zones, [(0, 0), goal])
    res = plan_route_3d(world, (0, 0), goal, algorithm, cruise)
    if res is None:
        raise RuntimeError("No safe path found. Try 3D A*, a different cruise altitude, or fewer zones.")
    return FlightPlan(Scene(origin, world, res.path, zones, buildings), calculate_flight_metrics(res.path, world, speed),
                      explain(res.path, world, res), len(buildings), algorithm)


# ---------------- side panel ----------------
with st.sidebar:
    st.header("Mission setup")
    start = st.text_input("Start address (Point A)", CFG["default_addresses"]["start"])
    dest = st.text_input("Destination address (Point B)", CFG["default_addresses"]["destination"])
    demo = st.checkbox("Offline demo city (no internet needed)")
    algorithm = st.selectbox("AI algorithm", list(ALGORITHMS), format_func=ALGORITHMS.get)
    cruise = st.select_slider("Cruise altitude (2D baselines only)", [30, 50, 70, 90, 110, 130, 150], 70,
                              disabled=algorithm == "astar3d")
    speed = st.number_input("Drone speed (m/s)", 1.0, 40.0, 12.0)
    clearance = st.slider("Roof clearance buffer (m)", 5, 30, 10)
    n_zones = st.slider("No-fly zones", 2, 3, 3)
    if st.button("🚀 Plan Route", type="primary", use_container_width=True):
        try:
            with st.spinner("Geocoding, loading buildings, planning…"):
                st.session_state.plan = run_planning(start, dest, demo, algorithm, cruise, clearance, n_zones, speed)
                st.session_state.progress = 0.0
        except Exception as e:
            st.session_state.pop("plan", None)
            st.error(str(e))
    st.header("Camera")
    mode = st.radio("View", ["Orbit", "FPV"], horizontal=True, help="Orbit = free-roam. FPV = chase camera behind the drone.")
    bearing = st.slider("Orbit rotation", -180, 180, -20, disabled=mode == "FPV")
    pitch = st.slider("Orbit tilt", 0, 85, 60, disabled=mode == "FPV")

plan = st.session_state.get("plan")
if not plan:
    st.info("Enter two addresses (or tick the offline demo) and press **Plan Route** in the sidebar.")
    st.stop()

scene, m = plan.scene, plan.metrics
main, side = st.columns([3, 1.25])
with side:
    inset = st.empty()                                           # 2D overview inset
    st.subheader("AI Path Dashboard")
    a, b = st.columns(2)
    a.metric("Distance", f"{m['dist'] / 1000:.2f} km"); b.metric("Flight time", f"{int(m['seconds'] // 60)}m {int(m['seconds'] % 60)}s")
    a.metric("Battery use", f"{m['battery']:.1f} %"); b.metric("Safety score", f"{m['score']} / 100")
    st.caption(f"{ALGORITHMS[plan.algorithm]} · {plan.n_buildings} buildings · min roof clearance "
               f"{m['min_vert']:.0f} m · min zone gap {m['min_nfz']:.0f} m")
with main:
    scene_slot = st.empty()                                      # main 3D viewport
    c1, c2 = st.columns([1, 4])
    fly = c1.button("▶ Fly Drone", use_container_width=True)
    prog = c2.slider("Flight progress", 0.0, 1.0, key="progress", step=0.01, label_visibility="collapsed")


def render(p):
    scene_slot.pydeck_chart(scene.deck3d(p, mode, bearing, pitch), height=640)
    inset.pydeck_chart(scene.deck2d(p), height=260)


render(prog)
if fly:                                                          # server-driven animation, one update per frame
    for p in np.linspace(0, 1, 50):
        render(float(p)); time.sleep(0.05)

with st.expander("🧠 AI Explainer Log - why this route?", expanded=True):
    for line in plan.log:
        st.markdown(f"- {line}")
