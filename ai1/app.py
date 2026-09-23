"""
app.py
Streamlit front-end for the AI Delivery Drone Path Planning simulation,
using a real interactive map for start/goal selection and a true 3D map
(extruded buildings, tilted route with altitude drop-lines) for results.

Run with:
    streamlit run app.py
"""

import math
import time
import numpy as np
import streamlit as st
import pydeck as pdk
import folium
from streamlit_folium import st_folium

from environment import AirspaceEnvironment, DynamicObstacle
from heuristics import MultiFactorHeuristic, DronePhysicsModel
from planner import AStar3DPlanner, ReactiveMicroAdjuster

st.set_page_config(page_title="Drone Path Planning", page_icon="🚁", layout="wide")

st.markdown(
    """
    <style>
    .hero {
        padding: 1.1rem 1.4rem; border-radius: 14px;
        background: linear-gradient(135deg, #0f2027 0%, #203a43 50%, #2c5364 100%);
        color: #ffffff; margin-bottom: 1.2rem;
    }
    .hero h1 { margin: 0; font-size: 1.6rem; }
    .hero p { margin: 0.3rem 0 0 0; opacity: 0.85; font-size: 0.95rem; }
    .legend-dot {
        display: inline-block; width: 10px; height: 10px; border-radius: 50%;
        margin-right: 6px; vertical-align: middle;
    }
    </style>
    <div class="hero">
        <h1>🚁 AI Delivery Drone Path Planning</h1>
        <p>Click a real map to set Start &amp; Goal, then fly a 3D A* route with reactive obstacle avoidance — buildings, no-fly zones and altitude are all rendered in true 3D.</p>
    </div>
    """,
    unsafe_allow_html=True,
)

EARTH_R = 6371000.0  # meters

# ---------------------------------------------------------------------------
# Session state
# ---------------------------------------------------------------------------
st.session_state.setdefault("start_latlon", None)
st.session_state.setdefault("goal_latlon", None)
st.session_state.setdefault("mode", "Start")
st.session_state.setdefault("last_click_id", None)


def latlon_to_local(lat, lon, lat0, lon0):
    dx = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    dy = math.radians(lat - lat0) * EARTH_R
    return dx, dy


def local_to_latlon(dx, dy, lat0, lon0):
    lat = lat0 + math.degrees(dy / EARTH_R)
    lon = lon0 + math.degrees(dx / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


# ---------------------------------------------------------------------------
# Sidebar controls
# ---------------------------------------------------------------------------
with st.sidebar:
    st.header("✈️ Point Selection")
    st.radio("Clicking the map sets:", ["Start", "Goal"], key="mode", horizontal=True)
    c1, c2 = st.columns(2)
    if c1.button("Reset Start", use_container_width=True):
        st.session_state.start_latlon = None
    if c2.button("Reset Goal", use_container_width=True):
        st.session_state.goal_latlon = None

    with st.expander("🛫 Altitudes (meters)", expanded=True):
        start_alt = st.slider("Start altitude", 0, 200, 10, 5)
        goal_alt = st.slider("Goal altitude", 0, 200, 30, 5)

    with st.expander("🔋 Drone Physics & Wind"):
        base_weight = st.slider("Base weight (kg)", 0.5, 10.0, 3.5, 0.5)
        payload_weight = st.slider("Payload weight (kg)", 0.0, 10.0, 2.0, 0.5)
        wx = st.slider("Wind X (m/s)", -5.0, 5.0, -2.0, 0.5)
        wy = st.slider("Wind Y (m/s)", -5.0, 5.0, 1.5, 0.5)
        wz = st.slider("Wind Z (m/s)", -5.0, 5.0, 0.0, 0.5)

    with st.expander("🧭 Heuristic Weights"):
        w_dist = st.slider("Distance weight", 0.0, 5.0, 1.0, 0.1)
        w_energy = st.slider("Energy weight", 0.0, 5.0, 1.8, 0.1)
        w_risk = st.slider("Risk weight", 0.0, 5.0, 3.0, 0.1)

    with st.expander("🏢 Obstacles", expanded=True):
        add_obstacles = st.checkbox("Add sample buildings / no-fly zone / moving obstacles", value=True)

    st.divider()
    run_button = st.button("🚀 Run Simulation", type="primary", use_container_width=True)


# ---------------------------------------------------------------------------
# Step 1: click-to-place map + status checklist
# ---------------------------------------------------------------------------
st.subheader("1️⃣ Pick Start and Goal on the map")

status1, status2 = st.columns(2)
if st.session_state.start_latlon:
    status1.success(f"✅ Start set — {st.session_state.start_latlon[0]:.5f}, {st.session_state.start_latlon[1]:.5f}")
else:
    status1.warning("⬜ Start not set")
if st.session_state.goal_latlon:
    status2.success(f"✅ Goal set — {st.session_state.goal_latlon[0]:.5f}, {st.session_state.goal_latlon[1]:.5f}")
else:
    status2.warning("⬜ Goal not set")

st.caption(f"Current click mode: **{st.session_state.mode}** — change it in the sidebar, then click the map.")

center = [17.3850, 78.4867]
if st.session_state.start_latlon:
    center = list(st.session_state.start_latlon)

fmap = folium.Map(location=center, zoom_start=13, tiles="OpenStreetMap")
if st.session_state.start_latlon:
    folium.Marker(st.session_state.start_latlon, tooltip="Start (Warehouse)",
                  icon=folium.Icon(color="green", icon="play")).add_to(fmap)
if st.session_state.goal_latlon:
    folium.Marker(st.session_state.goal_latlon, tooltip="Goal (Customer)",
                  icon=folium.Icon(color="red", icon="flag")).add_to(fmap)

map_data = st_folium(fmap, height=430, use_container_width=True, key="picker_map")

if map_data and map_data.get("last_clicked"):
    click = map_data["last_clicked"]
    click_id = (round(click["lat"], 6), round(click["lng"], 6))
    if click_id != st.session_state.last_click_id:
        st.session_state.last_click_id = click_id
        if st.session_state.mode == "Start":
            st.session_state.start_latlon = (click["lat"], click["lng"])
        else:
            st.session_state.goal_latlon = (click["lat"], click["lng"])
        st.rerun()


# ---------------------------------------------------------------------------
# Simulation helpers
# ---------------------------------------------------------------------------
def build_environment(bounds, wind_vector, grid_start, grid_goal, add_obs):
    env = AirspaceEnvironment(bounds, wind_vector=wind_vector)
    if not add_obs:
        return env

    bx, by, bz = bounds
    mid1 = (int(grid_start[0] + (grid_goal[0] - grid_start[0]) * 0.33),
            int(grid_start[1] + (grid_goal[1] - grid_start[1]) * 0.33))
    mid2 = (int(grid_start[0] + (grid_goal[0] - grid_start[0]) * 0.66),
            int(grid_start[1] + (grid_goal[1] - grid_start[1]) * 0.66))
    span = max(2, bx // 15)

    env.add_static_building(
        min_corner=(max(0, mid1[0] - span), max(0, mid1[1] - span), 0),
        max_corner=(min(bx - 1, mid1[0] + span), min(by - 1, mid1[1] + span), min(bz - 1, bz // 2)),
    )
    env.add_no_fly_zone(
        center=(min(bx - 1, mid2[0]), min(by - 1, mid2[1]), 0),
        radius=max(2, span), height_range=(0, bz - 1),
    )
    env.dynamic_obstacles.append(DynamicObstacle(
        start_pos=(min(bx - 1, mid1[0]), min(by - 1, mid1[1] + span), min(bz - 1, bz // 2)), velocity=(0, 1, 0)))
    env.dynamic_obstacles.append(DynamicObstacle(
        start_pos=(min(bx - 1, mid2[0] + span), min(by - 1, mid2[1]), min(bz - 1, bz // 3)), velocity=(-1, 0, 0)))
    return env


def run_simulation(start_latlon, goal_latlon, start_alt_m, goal_alt_m):
    lat0, lon0 = start_latlon
    dx, dy = latlon_to_local(goal_latlon[0], goal_latlon[1], lat0, lon0)
    horiz_dist = max(math.hypot(dx, dy), 1.0)

    cell_size_m = max(horiz_dist / 35.0, 1.0)
    z_cell_m = 5.0
    margin = 8

    gx_goal = int(round(dx / cell_size_m))
    gy_goal = int(round(dy / cell_size_m))

    grid_start = [margin, margin, int(round(start_alt_m / z_cell_m)) + margin]
    grid_goal = [margin + gx_goal, margin + gy_goal, int(round(goal_alt_m / z_cell_m)) + margin]
    if gx_goal < 0:
        grid_start[0] -= gx_goal
        grid_goal[0] -= gx_goal
    if gy_goal < 0:
        grid_start[1] -= gy_goal
        grid_goal[1] -= gy_goal
    grid_start, grid_goal = tuple(grid_start), tuple(grid_goal)

    bounds = (abs(gx_goal) + 2 * margin + 1, abs(gy_goal) + 2 * margin + 1,
              max(grid_start[2], grid_goal[2]) + margin)

    env = build_environment(bounds, (wx, wy, wz), grid_start, grid_goal, add_obstacles)
    physics = DronePhysicsModel(base_weight=base_weight, payload_weight=payload_weight)
    heuristic = MultiFactorHeuristic(w_dist=w_dist, w_energy=w_energy, w_risk=w_risk)

    t0 = time.perf_counter()
    macro_path = AStar3DPlanner(env, physics, heuristic).plan_path(grid_start, grid_goal)
    planner_ms = (time.perf_counter() - t0) * 1000
    if not macro_path:
        return None

    micro_adjuster = ReactiveMicroAdjuster(env)
    actual_path = [grid_start]
    current_pos = grid_start
    total_energy = 0.0

    t1 = time.perf_counter()
    for wp in macro_path[1:]:
        env.update_dynamic_environment()
        risk = env.get_dynamic_repulsion(current_pos, influence_radius=2.5)
        next_step = micro_adjuster.compute_next_step(current_pos, wp) if risk > 0.5 else wp
        total_energy += physics.calculate_step_energy(current_pos, next_step, env.wind_vector)
        current_pos = next_step
        actual_path.append(current_pos)
    sim_ms = (time.perf_counter() - t1) * 1000

    total_distance = sum(
        np.linalg.norm(np.array(actual_path[i]) - np.array(actual_path[i - 1]))
        for i in range(1, len(actual_path))
    ) * cell_size_m

    def grid_to_geo(x, y, z):
        lat, lon = local_to_latlon((x - grid_start[0]) * cell_size_m, (y - grid_start[1]) * cell_size_m, lat0, lon0)
        return lat, lon, z * z_cell_m

    macro_geo = [grid_to_geo(*p) for p in macro_path]
    actual_geo = [grid_to_geo(*p) for p in actual_path]

    # Collapse building voxels into footprints (one extruded column per x,y, height = tallest voxel)
    footprints = {}
    for (x, y, z) in env.static_obstacles:
        footprints[(x, y)] = max(footprints.get((x, y), -1), z)
    buildings = []
    for (x, y), top_z in footprints.items():
        lat, lon, _ = grid_to_geo(x, y, 0)
        buildings.append({"lat": lat, "lon": lon, "height_m": (top_z + 1) * z_cell_m})

    obstacles_geo = [grid_to_geo(*[int(v) for v in obs.position]) for obs in env.dynamic_obstacles]

    metrics = {
        "success": actual_path[-1] == grid_goal,
        "distance_m": total_distance,
        "energy": total_energy,
        "planner_ms": planner_ms,
        "sim_ms": sim_ms,
        "waypoints": len(macro_path),
        "max_alt_m": max(p[2] for p in actual_geo),
    }
    return macro_geo, actual_geo, buildings, obstacles_geo, metrics, cell_size_m


def make_3d_deck(macro_geo, actual_geo, buildings, obstacles_geo, start_latlon, goal_latlon, building_radius_m):
    lat0, lon0 = start_latlon

    layers = [
        pdk.Layer("PathLayer",
                  data=[{"path": [[lon, lat, alt] for lat, lon, alt in macro_geo]}],
                  get_path="path", get_color=[40, 90, 220], width_min_pixels=2),
        pdk.Layer("PathLayer",
                  data=[{"path": [[lon, lat, alt] for lat, lon, alt in actual_geo]}],
                  get_path="path", get_color=[220, 40, 40], width_min_pixels=5),
    ]

    # Vertical "drop lines" from the flown path down to the ground so altitude is unambiguous
    drop_lines = [
        {"from": [lon, lat, alt], "to": [lon, lat, 0]}
        for i, (lat, lon, alt) in enumerate(actual_geo) if i % 2 == 0 and alt > 0.5
    ]
    if drop_lines:
        layers.append(pdk.Layer(
            "LineLayer", data=drop_lines,
            get_source_position="from", get_target_position="to",
            get_color=[150, 150, 150, 120], get_width=1,
        ))

    if buildings:
        layers.append(pdk.Layer(
            "ColumnLayer",
            data=[{"position": [b["lon"], b["lat"]], "elevation": b["height_m"]} for b in buildings],
            get_position="position", get_elevation="elevation", elevation_scale=1,
            radius=max(building_radius_m * 0.9, 2),
            get_fill_color=[120, 120, 130, 200], pickable=True, auto_highlight=True,
        ))

    if obstacles_geo:
        layers.append(pdk.Layer(
            "ScatterplotLayer",
            data=[{"position": [lon, lat, alt]} for lat, lon, alt in obstacles_geo],
            get_position="position", get_fill_color=[160, 0, 200], get_radius=8,
        ))

    layers.append(pdk.Layer(
        "ScatterplotLayer",
        data=[{"position": [start_latlon[1], start_latlon[0], 0], "color": [0, 170, 0]},
              {"position": [goal_latlon[1], goal_latlon[0], 0], "color": [230, 180, 0]}],
        get_position="position", get_fill_color="color", get_radius=15,
    ))

    view_state = pdk.ViewState(latitude=lat0, longitude=lon0, zoom=14.2, pitch=60, bearing=15)
    return pdk.Deck(layers=layers, initial_view_state=view_state, map_style=None,
                     tooltip={"text": "Building height: {elevation} m"})


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
if run_button:
    if not st.session_state.start_latlon or not st.session_state.goal_latlon:
        st.error("Please set both a Start and a Goal point on the map first.")
    else:
        with st.spinner("Computing global path and simulating flight..."):
            result = run_simulation(st.session_state.start_latlon, st.session_state.goal_latlon, start_alt, goal_alt)

        if result is None:
            st.error("Global path planner failed to find a valid trajectory between start and goal.")
        else:
            macro_geo, actual_geo, buildings, obstacles_geo, metrics, cell_size_m = result

            st.subheader("2️⃣ Results — 3D Flight View")
            c1, c2, c3, c4, c5 = st.columns(5)
            c1.metric("Delivery", "SUCCESS" if metrics["success"] else "FAILED")
            c2.metric("Flight Distance", f"{metrics['distance_m']:.1f} m")
            c3.metric("Max Altitude", f"{metrics['max_alt_m']:.0f} m")
            c4.metric("Energy Consumed", f"{metrics['energy']:.2f} J")
            c5.metric("Planner Time", f"{metrics['planner_ms']:.2f} ms")

            deck = make_3d_deck(macro_geo, actual_geo, buildings, obstacles_geo,
                                 st.session_state.start_latlon, st.session_state.goal_latlon,
                                 building_radius_m=cell_size_m)
            st.pydeck_chart(deck, use_container_width=True)

            st.markdown(
                """
                <span class="legend-dot" style="background:#2858DC"></span>Planned A* path&nbsp;&nbsp;
                <span class="legend-dot" style="background:#DC2828"></span>Actual flown path&nbsp;&nbsp;
                <span class="legend-dot" style="background:#787882"></span>Buildings (true height)&nbsp;&nbsp;
                <span class="legend-dot" style="background:#A000C8"></span>Moving obstacles&nbsp;&nbsp;
                <span class="legend-dot" style="background:#00AA00"></span>Start&nbsp;&nbsp;
                <span class="legend-dot" style="background:#E6B400"></span>Goal
                """,
                unsafe_allow_html=True,
            )
            st.caption("Drag to rotate/tilt the map. Thin gray lines drop straight down from the flight path to the ground so altitude is easy to read at a glance.")
else:
    st.info("Set Start and Goal on the map above, then click **🚀 Run Simulation** in the sidebar.")
