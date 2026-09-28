"""Delivery drone path planner - Streamlit app.

Local:   streamlit run app.py
Cloud:   set the main file to app.py (planner.py must sit next to it)
"""
import streamlit as st
import streamlit.components.v1 as components

from planner import DEFAULT_OBSTACLES, SIZE, plan, rectangle

CELL = 12  # pixels per cell in the map drawing
NAMES = {"astar": "A* (fast)", "dijkstra": "Dijkstra (baseline)"}

st.set_page_config(page_title="Drone Path Planner", page_icon="🚁", layout="wide")
st.title("🚁 Delivery Drone Path Planner")

if "obstacles" not in st.session_state:
    st.session_state.obstacles = set(DEFAULT_OBSTACLES)


def draw_map(obstacles, route, start, goal):
    """The map as an SVG string, so no plotting library is needed."""
    px = SIZE * CELL
    half = CELL / 2
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{px}" height="{px}" '
             f'viewBox="0 0 {px} {px}" style="background:#0b1322;border:2px solid #334155">']
    for i in range(SIZE + 1):
        parts.append(f'<line x1="{i * CELL}" y1="0" x2="{i * CELL}" y2="{px}" stroke="#1e293b"/>')
        parts.append(f'<line x1="0" y1="{i * CELL}" x2="{px}" y2="{i * CELL}" stroke="#1e293b"/>')
    for x, y in obstacles:
        parts.append(f'<rect x="{x * CELL}" y="{y * CELL}" width="{CELL}" height="{CELL}" fill="#f59e0b"/>')
    if route:
        points = " ".join(f"{x * CELL + half},{y * CELL + half}" for x, y in route)
        parts.append(f'<polyline points="{points}" fill="none" stroke="#38bdf8" stroke-width="4"/>')
    for (x, y), color in ((start, "#22c55e"), (goal, "#ef4444")):
        parts.append(f'<circle cx="{x * CELL + half}" cy="{y * CELL + half}" r="7" fill="{color}"/>')
    parts.append("</svg>")
    return "".join(parts)


# ------------------------------------------------------------- sidebar
with st.sidebar:
    st.header("Mission")
    c1, c2 = st.columns(2)
    sx = c1.number_input("Start X", 0, SIZE - 1, 2, 1)
    sy = c2.number_input("Start Y", 0, SIZE - 1, 2, 1)
    dx = c1.number_input("Destination X", 0, SIZE - 1, 45, 1)
    dy = c2.number_input("Destination Y", 0, SIZE - 1, 45, 1)
    speed = st.number_input("Drone speed (m/s)", 1.0, 100.0, 10.0, 1.0)
    algorithm = st.selectbox("Algorithm", list(NAMES), format_func=NAMES.get)

    st.header("Obstacles")
    with st.form("block"):
        st.caption("Corners of a rectangle (cells).")
        a, b = st.columns(2)
        x1 = a.number_input("X from", 0, SIZE - 1, 20, 1)
        y1 = b.number_input("Y from", 0, SIZE - 1, 30, 1)
        x2 = a.number_input("X to", 0, SIZE - 1, 25, 1)
        y2 = b.number_input("Y to", 0, SIZE - 1, 35, 1)
        add = st.form_submit_button("Add block")
        remove = st.form_submit_button("Remove block")
    if add:
        st.session_state.obstacles |= rectangle(x1, y1, x2, y2)
    if remove:
        st.session_state.obstacles -= rectangle(x1, y1, x2, y2)

    r1, r2 = st.columns(2)
    if r1.button("Reset map"):
        st.session_state.obstacles = set(DEFAULT_OBSTACLES)
    if r2.button("Clear all"):
        st.session_state.obstacles = set()
    st.caption("One cell is 10 m.")

# ---------------------------------------------------------------- main
blocked = st.session_state.obstacles
start, goal = (int(sx), int(sy)), (int(dx), int(dy))

results, error = {}, None
try:
    for name in NAMES:
        results[name] = plan(start, goal, blocked, name, float(speed))
except ValueError as e:
    error = str(e)

chosen = results.get(algorithm)
map_col, info_col = st.columns([3, 2])

with map_col:
    components.html(draw_map(blocked, chosen["route"] if chosen else None, start, goal),
                    height=SIZE * CELL + 10)

with info_col:
    if error:
        st.error(error)
    else:
        st.subheader(NAMES[algorithm])
        m1, m2 = st.columns(2)
        m1.metric("Distance", f"{chosen['distance_m']} m")
        m2.metric("Flight time", f"{chosen['flight_time']} s")
        m3, m4 = st.columns(2)
        m3.metric("Waypoints", chosen["waypoints"])
        m4.metric("Cells searched", chosen["cells_searched"])

        st.subheader("A* vs Dijkstra")
        st.table({
            "Algorithm": [NAMES[n] for n in results],
            "Distance (m)": [r["distance_m"] for r in results.values()],
            "Cells searched": [r["cells_searched"] for r in results.values()],
        })
        st.caption("Both find the same shortest route; A* searches far fewer cells.")
