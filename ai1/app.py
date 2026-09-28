"""AI Delivery Drone Path Planner - Streamlit app.  Run: streamlit run app.py"""
import math, time
import numpy as np
import streamlit as st

from drone import geo, planner as P
from drone.scene import Scene

st.set_page_config(page_title="AI Drone Path Planner", page_icon="🚁", layout="wide")
st.title("🚁 AI-Based Delivery Drone Path Planner")


@st.cache_data(show_spinner=False, ttl=3600)
def cached_buildings(origin, bounds):
    return geo.fetch_buildings(origin, bounds)


def run_planning(start, dest, demo, clearance, n_zones):
    """Geocode -> zones -> buildings -> collision world -> A* -> metrics -> explainer log."""
    if demo:
        origin, goal = (78.3800, 17.4435), (850.0, 350.0)
    else:
        origin = geo.geocode(start)
        goal = tuple(float(v) for v in geo.to_local(origin, *geo.geocode(dest)))
    L = math.hypot(*goal)
    if not 200 <= L <= 4000:
        raise ValueError(f"Points are {L:.0f} m apart - choose 200 m to 4 km.")
    zones = P.make_zones(goal, n_zones)
    xs = [0, goal[0]] + [z.x + s * z.r for z in zones for s in (-1, 1)]
    ys = [0, goal[1]] + [z.y + s * z.r for z in zones for s in (-1, 1)]
    bounds = (min(xs) - 250, min(ys) - 250, max(xs) + 250, max(ys) + 250)
    buildings = geo.demo_buildings(bounds, [(0, 0), goal]) if demo else cached_buildings(origin, bounds)
    cell = max(30.0, max(bounds[2] - bounds[0], bounds[3] - bounds[1]) / 90)
    world = P.World(bounds, cell, clearance, buildings, zones, [(0, 0), goal])
    res = P.plan(world, (0, 0), goal)
    if res is None:
        raise RuntimeError("No safe path found (buildings too tall or a pad is enclosed by zones).")
    return dict(scene=Scene(origin, world, res.path, zones, buildings), metrics=P.metrics(res.path, world),
                log=P.explain(res.path, world, res), n_buildings=len(buildings))


# ---------------- side panel: inputs ----------------
with st.sidebar:
    st.header("Mission")
    start = st.text_input("Start address (Point A)", "Cyber Towers, HITEC City, Hyderabad")
    dest = st.text_input("Destination address (Point B)", "Inorbit Mall, Madhapur, Hyderabad")
    demo = st.checkbox("Offline demo city (no internet needed)", value=False)
    clearance = st.slider("Roof clearance buffer (m)", 5, 30, 10)
    n_zones = st.slider("No-fly zones", 2, 3, 3)
    if st.button("Plan Route", type="primary", use_container_width=True):
        try:
            with st.spinner("Geocoding, loading buildings, running 3D A*…"):
                st.session_state.plan = run_planning(start, dest, demo, clearance, n_zones)
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

scene, m = plan["scene"], plan["metrics"]
main, side = st.columns([3, 1.25])

# ---------------- right column: 2D inset + dashboard ----------------
with side:
    inset = st.empty()
    st.subheader("AI Path Dashboard")
    a, b = st.columns(2)
    a.metric("Distance", f"{m['dist'] / 1000:.2f} km"); b.metric("Flight time", f"{int(m['seconds'] // 60)}m {int(m['seconds'] % 60)}s")
    a.metric("Battery use", f"{m['battery']:.1f} %"); b.metric("Safety score", f"{m['score']} / 100")
    st.caption(f"{plan['n_buildings']} buildings scanned · min roof clearance {m['min_vert']:.0f} m · min zone gap {m['min_nfz']:.0f} m")

# ---------------- main column: 3D viewport + playback ----------------
with main:
    scene_slot = st.empty()
    c1, c2 = st.columns([1, 4])
    fly = c1.button("▶ Fly Drone", use_container_width=True)
    prog = c2.slider("Flight progress", 0.0, 1.0, key="progress", step=0.01, label_visibility="collapsed")


def render(p):
    scene_slot.pydeck_chart(scene.deck3d(p, mode, bearing, pitch), height=640)
    inset.pydeck_chart(scene.deck2d(p), height=260)


render(prog)
if fly:                                              # server-driven animation: one map update per frame
    for p in np.linspace(0, 1, 50):
        render(float(p)); time.sleep(0.05)

with st.expander("🧠 AI Explainer Log - why this route?", expanded=True):
    for line in plan["log"]:
        st.markdown(f"- {line}")
