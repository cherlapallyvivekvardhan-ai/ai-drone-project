"""
app.py
Streamlit front-end for the AI Delivery Drone Path Planning simulation,
using a real interactive map for start/goal selection and a 3D map view
for the resulting flight path.

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

st.set_page_config(page_title="Drone Path Planning", layout="wide")
st.title("AI Delivery Drone Path Planning Simulation")
st.caption("Click the map to place a Start and Goal point, then run the 3D A* + reactive-avoidance planner.")

EARTH_R = 6371000.0  # meters

# ---------------------------------------------------------------------------
# Session state
# ---------------------------------------------------------------------------
st.session_state.setdefault("start_latlon", None)
st.session_state.setdefault("goal_latlon", None)
st.session_state.setdefault("mode", "Start")
st.session_state.setdefault("last_click_id", None)


# ---------------------------------------------------------------------------
# Coordinate conversion (equirectangular local ENU, fine for city-scale spans)
# ---------------------------------------------------------------------------
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
    st.header("Point Selection")
    st.radio("Clicking the map sets:", ["Start", "Goal"], key="mode")
    c1, c2 = st.columns(2)
    if c1.button("Reset Start"):
        st.session_state.start_latlon = None
    if c2.button("Reset Goal"):
        st.session_state.goal_latlon = None

    st.divider()
    st.header("Flight Altitudes (meters)")
    start_alt = st.slider("Start altitude", 0, 200, 10, 5)
    goal_alt = st.slider("Goal altitude", 0, 200, 30, 5)

    st.header("Drone Physics")
    base_weight = st.slider("Base weight (kg)", 0.5, 10.0, 3.5, 0.5)
    payload_weight = st.slider("Payload weight (kg)", 0.0, 10.0, 2.0, 0.5)

    st.header("Wind (m/s)")
    wx = st.slider("Wind X", -5.0, 5.0, -2.0, 0.5)
    wy = st.slider("Wind Y", -5.0, 5.0, 1.5, 0.5)
    wz = st.slider("Wind Z", -5.0, 5.0, 0.0, 0.5)

    st.header("Heuristic Weights")
    w_dist = st.slider("Distance weight", 0.0, 5.0, 1.0, 0.1)
    w_energy = st.slider("Energy weight", 0.0, 5.0, 1.8, 0.1)
    w_risk = st.slider("Risk weight", 0.0, 5.0, 3.0, 0.1)

    st.header("Obstacles")
    add_obstacles = st.checkbox("Add sample buildings / no-fly zone / moving obstacles along route", value=True)

    run_button = st.button("Run Simulation", type="primary", use_container_width=True)


# ---------------------------------------------------------------------------
# Step 1: click-to-place map
# ---------------------------------------------------------------------------
st.subheader("1. Pick Start and Goal on the map")
st.write(f"Current mode: **{st.session_state.mode}** — click the map to set that point.")

center = [17.3850, 78.4867]  # default view (Hyderabad)
if st.session_state.start_latlon:
    center = list(st.session_state.start_latlon)

fmap = folium.Map(location=center, zoom_start=13, tiles="OpenStreetMap")

if st.session_state.start_latlon:
    folium.Marker(
        st.session_state.start_latlon, tooltip="Start (Warehouse)",
        icon=folium.Icon(color="green", icon="play"),
    ).add_to(fmap)

if st.session_state.goal_latlon:
    folium.Marker(
        st.session_state.goal_latlon, tooltip="Goal (Customer)",
        icon=folium.Icon(color="red", icon="flag"),
    ).add_to(fmap)

map_data = st_folium(fmap, height=450, use_container_width=True, key="picker_map")

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

colA, colB = st.columns(2)
colA.info(f"Start: {st.session_state.start_latlon or 'not set'}")
colB.info(f"Goal: {st.session_state.goal_latlon or 'not set'}")


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------
def build_environment(bounds, wind_vector, grid_start, grid_goal, add_obs):
    env = AirspaceEnvironment(bounds, wind_vector=wind_vector)
    if not add_obs:
        return env

    bx, by, bz = bounds
    mid1 = (
        int(grid_start[0] + (grid_goal[0] - grid_start[0]) * 0.33),
        int(grid_start[1] + (grid_goal[1] - grid_start[1]) * 0.33),
    )
    mid2 = (
        int(grid_start[0] + (grid_goal[0] - grid_start[0]) * 0.66),
        int(grid_start[1] + (grid_goal[1] - grid_start[1]) * 0.66),
    )
    span = max(2, bx // 15)

    env.add_static_building(
        min_corner=(max(0, mid1[0] - span), max(0, mid1[1] - span), 0),
        max_corner=(min(bx - 1, mid1[0] + span), min(by - 1, mid1[1] + span), min(bz - 1, bz // 2)),
    )
    env.add_no_fly_zone(
        center=(min(bx - 1, mid2[0]), min(by - 1, mid2[1]), 0),
        radius=max(2, span),
        height_range=(0, bz - 1),
    )
    env.dynamic_obstacles.append(
        DynamicObstacle(start_pos=(min(bx - 1, mid1[0]), min(by - 1, mid1[1] + span), min(bz - 1, bz // 2)), velocity=(0, 1, 0))
    )
    env.dynamic_obstacles.append(
        DynamicObstacle(start_pos=(min(bx - 1, mid2[0] + span), min(by - 1, mid2[1]), min(bz - 1, bz // 3)), velocity=(-1, 0, 0))
    )
    return env


def run_simulation(start_latlon, goal_latlon, start_alt_m, goal_alt_m):
    lat0, lon0 = start_latlon
    dx, dy = latlon_to_local(goal_latlon[0], goal_latlon[1], lat0, lon0)
    horiz_dist = max(math.hypot(dx, dy), 1.0)

    # Scale real-world meters down to a tractable grid (~35 cells across the route)
    cell_size_m = max(horiz_dist / 35.0, 1.0)
    z_cell_m = 5.0  # each vertical grid cell = 5 meters

    margin = 8
    gx_goal = int(round(dx / cell_size_m))
    gy_goal = int(round(dy / cell_size_m))

    grid_start = [margin, margin, int(round(start_alt_m / z_cell_m)) + margin]
    grid_goal = [
        margin + gx_goal,
        margin + gy_goal,
        int(round(goal_alt_m / z_cell_m)) + margin,
    ]
    # Shift both points so all coordinates stay non-negative
    if gx_goal < 0:
        grid_start[0] -= gx_goal
        grid_goal[0] -= gx_goal
    if gy_goal < 0:
        grid_start[1] -= gy_goal
        grid_goal[1] -= gy_goal
    grid_start = tuple(grid_start)
    grid_goal = tuple(grid_goal)

    bx = abs(gx_goal) + 2 * margin + 1
    by = abs(gy_goal) + 2 * margin + 1
    bz = max(grid_start[2], grid_goal[2]) + margin
    bounds = (bx, by, bz)

    env = build_environment(bounds, (wx, wy, wz), grid_start, grid_goal, add_obstacles)
    physics = DronePhysicsModel(base_weight=base_weight, payload_weight=payload_weight)
    heuristic = MultiFactorHeuristic(w_dist=w_dist, w_energy=w_energy, w_risk=w_risk)

    t0 = time.perf_counter()
    macro_planner = AStar3DPlanner(env, physics, heuristic)
    macro_path = macro_planner.plan_path(grid_start, grid_goal)
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
    ) * cell_size_m  # back to meters (approx.; ignores the separate z_cell_m scale)

    def grid_to_geo(p):
        gx, gy, gz = p
        lx = (gx - grid_start[0]) * cell_size_m
        ly = (gy - grid_start[1]) * cell_size_m
        lat, lon = local_to_latlon(lx, ly, lat0, lon0)
        alt_m = gz * z_cell_m
        return lat, lon, alt_m

    macro_geo = [grid_to_geo(p) for p in macro_path]
    actual_geo = [grid_to_geo(p) for p in actual_path]
    buildings_geo = [grid_to_geo((x, y, z)) for (x, y, z) in env.static_obstacles] if env.static_obstacles else []
    obstacles_geo = [grid_to_geo(tuple(int(v) for v in obs.position)) for obs in env.dynamic_obstacles]

    metrics = {
        "success": actual_path[-1] == grid_goal,
        "distance_m": total_distance,
        "energy": total_energy,
        "planner_ms": planner_ms,
        "sim_ms": sim_ms,
        "waypoints": len(macro_path),
    }
    return macro_geo, actual_geo, buildings_geo, obstacles_geo, metrics, cell_size_m


def make_3d_deck(macro_geo, actual_geo, buildings_geo, obstacles_geo, start_latlon, goal_latlon, building_radius_m):
    lat0, lon0 = start_latlon

    path_layer_actual = pdk.Layer(
        "PathLayer",
        data=[{"path": [[lon, lat, alt] for lat, lon, alt in actual_geo]}],
        get_path="path",
        get_color=[220, 40, 40],
        width_min_pixels=4,
        pickable=True,
    )
    path_layer_macro = pdk.Layer(
        "PathLayer",
        data=[{"path": [[lon, lat, alt] for lat, lon, alt in macro_geo]}],
        get_path="path",
        get_color=[40, 90, 220],
        width_min_pixels=2,
        pickable=True,
    )
    markers = [
        {"position": [start_latlon[1], start_latlon[0], 0], "color": [0, 160, 0]},
        {"position": [goal_latlon[1], goal_latlon[0], 0], "color": [230, 180, 0]},
    ]
    marker_layer = pdk.Layer(
        "ScatterplotLayer",
        data=markers,
        get_position="position",
        get_fill_color="color",
        get_radius=15,
        pickable=True,
    )

    layers = [path_layer_macro, path_layer_actual, marker_layer]

    if buildings_geo:
        b_data = [{"position": [lon, lat], "elevation": max(alt, 10)} for lat, lon, alt in buildings_geo]
        layers.append(pdk.Layer(
            "ColumnLayer",
            data=b_data,
            get_position="position",
            get_elevation="elevation",
            elevation_scale=1,
            radius=max(building_radius_m, 2),
            get_fill_color=[130, 130, 130, 150],
            pickable=False,
        ))

    if obstacles_geo:
        o_data = [{"position": [lon, lat, alt]} for lat, lon, alt in obstacles_geo]
        layers.append(pdk.Layer(
            "ScatterplotLayer",
            data=o_data,
            get_position="position",
            get_fill_color=[150, 0, 200],
            get_radius=8,
        ))

    view_state = pdk.ViewState(latitude=lat0, longitude=lon0, zoom=14, pitch=55, bearing=20)
    return pdk.Deck(layers=layers, initial_view_state=view_state, map_style=None)


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
if run_button:
    if not st.session_state.start_latlon or not st.session_state.goal_latlon:
        st.error("Please set both a Start and a Goal point on the map first.")
    else:
        with st.spinner("Computing global path and simulating flight..."):
            result = run_simulation(
                st.session_state.start_latlon, st.session_state.goal_latlon, start_alt, goal_alt
            )

        if result is None:
            st.error("Global path planner failed to find a valid trajectory between start and goal.")
        else:
            macro_geo, actual_geo, buildings_geo, obstacles_geo, metrics, cell_size_m = result

            st.subheader("2. Results")
            c1, c2, c3, c4, c5 = st.columns(5)
            c1.metric("Delivery", "SUCCESS" if metrics["success"] else "FAILED")
            c2.metric("Flight Distance", f"{metrics['distance_m']:.1f} m")
            c3.metric("Energy Consumed", f"{metrics['energy']:.2f} J")
            c4.metric("Planner Time", f"{metrics['planner_ms']:.2f} ms")
            c5.metric("Waypoints", metrics["waypoints"])

            deck = make_3d_deck(
                macro_geo, actual_geo, buildings_geo, obstacles_geo,
                st.session_state.start_latlon, st.session_state.goal_latlon,
                building_radius_m=cell_size_m,
            )
            st.pydeck_chart(deck, use_container_width=True)
            st.caption(
                "Blue = planned A* path · Red = actual flown path (with reactive avoidance) · "
                "Gray columns = buildings · Purple dots = moving obstacles"
            )
else:
    st.info("Set Start and Goal on the map above, then click **Run Simulation** in the sidebar.")
