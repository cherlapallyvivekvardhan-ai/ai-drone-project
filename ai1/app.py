"""
app.py
Streamlit front-end for the AI Delivery Drone Path Planning simulation.

Features:
 - Click a real map to set Start / Goal
 - Climb -> Cruise -> Descend altitude profile (drone height control)
 - True 3D map: height-colored extruded buildings, translucent no-fly cylinder,
   planned vs actual (re-routed) path, altitude drop-lines
 - Dynamic re-routing when a new obstacle appears mid-flight
 - Live animated playback of the flight on the 3D map
 - Analytics dashboard (altitude / energy / risk over the flight)
 - Comparison of this simulation against real drone-delivery systems

Run with:
    streamlit run app.py
"""

import math
import time
import numpy as np
import pandas as pd
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
    .hero { padding: 1.1rem 1.4rem; border-radius: 14px;
        background: linear-gradient(135deg, #0f2027 0%, #203a43 50%, #2c5364 100%);
        color: #fff; margin-bottom: 1.1rem; }
    .hero h1 { margin: 0; font-size: 1.6rem; }
    .hero p { margin: 0.3rem 0 0 0; opacity: 0.85; font-size: 0.95rem; }
    .legend-dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%;
        margin-right: 6px; vertical-align: middle; }
    </style>
    <div class="hero">
        <h1>🚁 AI Delivery Drone Path Planning</h1>
        <p>Click a real map to set Start &amp; Goal, control the drone's climb/cruise/descend height,
        fly a live-animated 3D route that re-plans around obstacles that appear mid-flight, and compare
        it against real-world delivery drones.</p>
    </div>
    """,
    unsafe_allow_html=True,
)

EARTH_R = 6371000.0  # meters

for key, default in [
    ("start_latlon", None), ("goal_latlon", None), ("mode", "Start"),
    ("last_click_id", None), ("sim_result", None),
]:
    st.session_state.setdefault(key, default)


def latlon_to_local(lat, lon, lat0, lon0):
    dx = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    dy = math.radians(lat - lat0) * EARTH_R
    return dx, dy


def local_to_latlon(dx, dy, lat0, lon0):
    lat = lat0 + math.degrees(dy / EARTH_R)
    lon = lon0 + math.degrees(dx / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def circle_polygon(lat0, lon0, radius_m, n=28):
    pts = []
    for i in range(n):
        ang = 2 * math.pi * i / n
        lat, lon = local_to_latlon(radius_m * math.cos(ang), radius_m * math.sin(ang), lat0, lon0)
        pts.append([lon, lat])
    return pts


def height_color(h, hmin, hmax):
    t = 0.0 if hmax <= hmin else (h - hmin) / (hmax - hmin)
    c_low, c_high = np.array([40, 170, 180]), np.array([230, 110, 30])
    c = c_low + t * (c_high - c_low)
    return [int(c[0]), int(c[1]), int(c[2]), 210]


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

    with st.expander("🛫 Drone Height Profile (climb → cruise → descend)", expanded=True):
        start_alt = st.slider("Start altitude (m)", 0, 200, 10, 5)
        cruise_alt = st.slider("Cruise altitude (m)", 10, 400, 80, 5,
                                help="The drone climbs to this height, flies the long leg at this height, then descends.")
        goal_alt = st.slider("Goal (delivery) altitude (m)", 0, 200, 15, 5)

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

    with st.expander("🏢 Obstacles & Dynamic Re-routing", expanded=True):
        add_obstacles = st.checkbox("Add sample buildings / no-fly zone / moving obstacles", value=True)
        inject_surprise = st.checkbox("🚨 Inject an unexpected obstacle mid-flight", value=True,
                                       help="Simulates a new hazard appearing after the flight has already started.")
        enable_reroute = st.checkbox("Enable dynamic re-routing", value=True)
        reroute_threshold = st.slider("Re-route sensitivity (lower = re-routes more easily)", 0.3, 3.0, 1.2, 0.1)

    with st.expander("🎬 Animation"):
        anim_speed = st.slider("Playback speed (seconds/step)", 0.02, 0.5, 0.08, 0.02)

    st.divider()
    run_button = st.button("🚀 Run Simulation", type="primary", use_container_width=True)


# ---------------------------------------------------------------------------
# Step 1: click-to-place map + status checklist
# ---------------------------------------------------------------------------
st.subheader("1️⃣ Pick Start and Goal on the map")

status1, status2 = st.columns(2)
if st.session_state.start_latlon:
    status1.success(f"✅ Start — {st.session_state.start_latlon[0]:.5f}, {st.session_state.start_latlon[1]:.5f}")
else:
    status1.warning("⬜ Start not set")
if st.session_state.goal_latlon:
    status2.success(f"✅ Goal — {st.session_state.goal_latlon[0]:.5f}, {st.session_state.goal_latlon[1]:.5f}")
else:
    status2.warning("⬜ Goal not set")

st.caption(f"Current click mode: **{st.session_state.mode}** — change it in the sidebar, then click the map.")

center = list(st.session_state.start_latlon) if st.session_state.start_latlon else [17.3850, 78.4867]
fmap = folium.Map(location=center, zoom_start=13, tiles="OpenStreetMap")
if st.session_state.start_latlon:
    folium.Marker(st.session_state.start_latlon, tooltip="Start (Warehouse)",
                  icon=folium.Icon(color="green", icon="play")).add_to(fmap)
if st.session_state.goal_latlon:
    folium.Marker(st.session_state.goal_latlon, tooltip="Goal (Customer)",
                  icon=folium.Icon(color="red", icon="flag")).add_to(fmap)

map_data = st_folium(fmap, height=420, use_container_width=True, key="picker_map")

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
# Environment + simulation
# ---------------------------------------------------------------------------
def build_environment(bounds, wind_vector, grid_start, grid_goal, add_obs):
    env = AirspaceEnvironment(bounds, wind_vector=wind_vector)
    if not add_obs:
        return env, None
    bx, by, bz = bounds
    mid1 = (int(grid_start[0] + (grid_goal[0] - grid_start[0]) * 0.33),
            int(grid_start[1] + (grid_goal[1] - grid_start[1]) * 0.33))
    mid2 = (int(grid_start[0] + (grid_goal[0] - grid_start[0]) * 0.66),
            int(grid_start[1] + (grid_goal[1] - grid_start[1]) * 0.66))
    span = max(2, bx // 15)

    env.add_static_building(
        min_corner=(max(0, mid1[0] - span), max(0, mid1[1] - span), 0),
        max_corner=(min(bx - 1, mid1[0] + span), min(by - 1, mid1[1] + span), min(bz - 1, bz // 2)))
    env.add_no_fly_zone(center=(min(bx - 1, mid2[0]), min(by - 1, mid2[1]), 0),
                         radius=max(2, span), height_range=(0, bz - 1))
    env.dynamic_obstacles.append(DynamicObstacle(
        start_pos=(min(bx - 1, mid1[0]), min(by - 1, mid1[1] + span), min(bz - 1, bz // 2)), velocity=(0, 1, 0)))
    env.dynamic_obstacles.append(DynamicObstacle(
        start_pos=(min(bx - 1, mid2[0] + span), min(by - 1, mid2[1]), min(bz - 1, bz // 3)), velocity=(-1, 0, 0)))
    no_fly_info = {"center_xy": mid2, "radius_cells": max(2, span), "height_cells": bz - 1}
    return env, no_fly_info


def plan_multi_leg(planner, p_start, p_goal, cruise_z):
    """Climb -> cruise -> descend; falls back to a direct plan if any leg fails."""
    top = max(cruise_z, p_start[2], p_goal[2])
    climb_top = (p_start[0], p_start[1], top)
    descend_top = (p_goal[0], p_goal[1], top)

    leg1 = planner.plan_path(p_start, climb_top)
    leg2 = planner.plan_path(climb_top, descend_top) if leg1 else None
    leg3 = planner.plan_path(descend_top, p_goal) if leg2 else None

    if leg1 and leg2 and leg3:
        return leg1 + leg2[1:] + leg3[1:]
    return planner.plan_path(p_start, p_goal)


def run_simulation(start_latlon, goal_latlon, start_alt_m, goal_alt_m, cruise_alt_m):
    lat0, lon0 = start_latlon
    dx, dy = latlon_to_local(goal_latlon[0], goal_latlon[1], lat0, lon0)
    horiz_dist = max(math.hypot(dx, dy), 1.0)

    cell_size_m = max(horiz_dist / 35.0, 1.0)
    z_cell_m = 5.0
    margin = 8

    gx_goal = int(round(dx / cell_size_m))
    gy_goal = int(round(dy / cell_size_m))
    z_start = int(round(start_alt_m / z_cell_m)) + margin
    z_goal = int(round(goal_alt_m / z_cell_m)) + margin
    z_cruise = int(round(cruise_alt_m / z_cell_m)) + margin

    grid_start = [margin, margin, z_start]
    grid_goal = [margin + gx_goal, margin + gy_goal, z_goal]
    if gx_goal < 0:
        grid_start[0] -= gx_goal
        grid_goal[0] -= gx_goal
    if gy_goal < 0:
        grid_start[1] -= gy_goal
        grid_goal[1] -= gy_goal
    grid_start, grid_goal = tuple(grid_start), tuple(grid_goal)

    bounds = (abs(gx_goal) + 2 * margin + 1, abs(gy_goal) + 2 * margin + 1,
              max(grid_start[2], grid_goal[2], z_cruise) + margin)

    env, no_fly_info = build_environment(bounds, (wx, wy, wz), grid_start, grid_goal, add_obstacles)
    physics = DronePhysicsModel(base_weight=base_weight, payload_weight=payload_weight)
    heuristic = MultiFactorHeuristic(w_dist=w_dist, w_energy=w_energy, w_risk=w_risk)
    macro_planner = AStar3DPlanner(env, physics, heuristic)

    t0 = time.perf_counter()
    macro_path = plan_multi_leg(macro_planner, grid_start, grid_goal, z_cruise)
    planner_ms = (time.perf_counter() - t0) * 1000
    if not macro_path:
        return None

    # Inject a surprise obstacle roughly halfway through the planned route
    if inject_surprise and len(macro_path) > 4:
        surprise_pos = macro_path[len(macro_path) // 2]
        env.dynamic_obstacles.append(DynamicObstacle(start_pos=surprise_pos, velocity=(0, 0, 0), radius=2.0))

    micro_adjuster = ReactiveMicroAdjuster(env)
    path = list(macro_path)
    i = 1
    current_pos = grid_start
    actual_path = [grid_start]
    trace = [{"step": 0, "alt_m": (grid_start[2] - margin) * z_cell_m, "energy_step": 0.0,
              "cum_energy": 0.0, "risk": 0.0}]
    replan_events = []
    total_energy = 0.0
    guard = 0
    guard_limit = 500

    t1 = time.perf_counter()
    while current_pos != grid_goal and i < len(path) and guard < guard_limit:
        guard += 1
        env.update_dynamic_environment()
        risk = env.get_dynamic_repulsion(current_pos, influence_radius=2.5)
        target = path[i]

        if enable_reroute and risk > reroute_threshold:
            new_tail = macro_planner.plan_path(current_pos, grid_goal)
            if new_tail and len(new_tail) > 1:
                path = new_tail
                i = 1
                replan_events.append({"step": guard, "pos": current_pos})
                target = path[i]

        if risk > 0.5:
            next_step = micro_adjuster.compute_next_step(current_pos, target)
        else:
            next_step = target

        step_energy = physics.calculate_step_energy(current_pos, next_step, env.wind_vector)
        total_energy += step_energy
        current_pos = next_step
        actual_path.append(current_pos)
        trace.append({"step": guard, "alt_m": (current_pos[2] - margin) * z_cell_m,
                      "energy_step": step_energy, "cum_energy": total_energy, "risk": risk})
        if current_pos == target:
            i += 1
    sim_ms = (time.perf_counter() - t1) * 1000

    total_distance = sum(
        np.linalg.norm(np.array(actual_path[k]) - np.array(actual_path[k - 1]))
        for k in range(1, len(actual_path))
    ) * cell_size_m

    def grid_to_geo(x, y, z):
        lat, lon = local_to_latlon((x - grid_start[0]) * cell_size_m, (y - grid_start[1]) * cell_size_m, lat0, lon0)
        return lat, lon, (z - margin) * z_cell_m

    macro_geo = [grid_to_geo(*p) for p in macro_path]
    actual_geo = [grid_to_geo(*p) for p in actual_path]
    replan_geo = [grid_to_geo(*e["pos"]) for e in replan_events]

    footprints = {}
    for (x, y, z) in env.static_obstacles:
        footprints[(x, y)] = max(footprints.get((x, y), -1), z)
    buildings = []
    for (x, y), top_z in footprints.items():
        lat, lon, _ = grid_to_geo(x, y, margin)
        buildings.append({"lat": lat, "lon": lon, "height_m": (top_z + 1) * z_cell_m})

    obstacles_geo = [grid_to_geo(*[int(v) for v in obs.position]) for obs in env.dynamic_obstacles]

    no_fly_poly, no_fly_height_m = None, 0.0
    if no_fly_info:
        cx, cy = no_fly_info["center_xy"]
        lat_c, lon_c, _ = grid_to_geo(cx, cy, margin)
        no_fly_poly = circle_polygon(lat_c, lon_c, no_fly_info["radius_cells"] * cell_size_m)
        no_fly_height_m = no_fly_info["height_cells"] * z_cell_m

    metrics = {
        "success": actual_path[-1] == grid_goal,
        "distance_m": total_distance,
        "energy": total_energy,
        "planner_ms": planner_ms,
        "sim_ms": sim_ms,
        "waypoints": len(macro_path),
        "max_alt_m": max(p[2] for p in actual_geo),
        "reroutes": len(replan_events),
    }
    return {
        "macro_geo": macro_geo, "actual_geo": actual_geo, "buildings": buildings,
        "obstacles_geo": obstacles_geo, "replan_geo": replan_geo, "no_fly_poly": no_fly_poly,
        "no_fly_height_m": no_fly_height_m, "metrics": metrics, "trace": trace,
        "start_latlon": start_latlon, "goal_latlon": goal_latlon, "cell_size_m": cell_size_m,
    }


def make_3d_deck(res, path_so_far=None, drone_pos=None):
    lat0, lon0 = res["start_latlon"]
    macro_geo, actual_geo = res["macro_geo"], res["actual_geo"]
    trail = path_so_far if path_so_far is not None else actual_geo

    layers = [
        pdk.Layer("PathLayer", data=[{"path": [[lon, lat, alt] for lat, lon, alt in macro_geo]}],
                  get_path="path", get_color=[40, 90, 220], width_min_pixels=2),
        pdk.Layer("PathLayer", data=[{"path": [[lon, lat, alt] for lat, lon, alt in trail]}],
                  get_path="path", get_color=[220, 40, 40], width_min_pixels=5),
    ]

    drop_lines = [{"from": [lon, lat, alt], "to": [lon, lat, 0]}
                  for idx, (lat, lon, alt) in enumerate(trail) if idx % 2 == 0 and alt > 0.5]
    if drop_lines:
        layers.append(pdk.Layer("LineLayer", data=drop_lines, get_source_position="from",
                                 get_target_position="to", get_color=[150, 150, 150, 120], get_width=1))

    if res["no_fly_poly"]:
        layers.append(pdk.Layer(
            "PolygonLayer", data=[{"polygon": res["no_fly_poly"]}], get_polygon="polygon",
            extruded=True, get_elevation=res["no_fly_height_m"], elevation_scale=1,
            get_fill_color=[255, 140, 0, 60], get_line_color=[255, 140, 0, 180], pickable=False))

    if res["buildings"]:
        heights = [b["height_m"] for b in res["buildings"]]
        hmin, hmax = min(heights), max(heights)
        b_data = [{"position": [b["lon"], b["lat"]], "elevation": b["height_m"],
                   "color": height_color(b["height_m"], hmin, hmax)} for b in res["buildings"]]
        layers.append(pdk.Layer(
            "ColumnLayer", data=b_data, get_position="position", get_elevation="elevation",
            elevation_scale=1, radius=max(res["cell_size_m"] * 0.9, 2),
            get_fill_color="color", pickable=True, auto_highlight=True))

    if res["obstacles_geo"]:
        layers.append(pdk.Layer(
            "ScatterplotLayer", data=[{"position": [lon, lat, alt]} for lat, lon, alt in res["obstacles_geo"]],
            get_position="position", get_fill_color=[160, 0, 200], get_radius=8))

    if res["replan_geo"]:
        layers.append(pdk.Layer(
            "ScatterplotLayer", data=[{"position": [lon, lat, alt]} for lat, lon, alt in res["replan_geo"]],
            get_position="position", get_fill_color=[255, 165, 0], get_radius=10))

    markers = [
        {"position": [res["start_latlon"][1], res["start_latlon"][0], 0], "color": [0, 170, 0]},
        {"position": [res["goal_latlon"][1], res["goal_latlon"][0], 0], "color": [230, 180, 0]},
    ]
    if drone_pos is not None:
        lat, lon, alt = drone_pos
        markers.append({"position": [lon, lat, alt], "color": [0, 220, 255]})
    layers.append(pdk.Layer("ScatterplotLayer", data=markers, get_position="position",
                             get_fill_color="color", get_radius=15))

    view_state = pdk.ViewState(latitude=lat0, longitude=lon0, zoom=14.2, pitch=60, bearing=15)
    return pdk.Deck(layers=layers, initial_view_state=view_state, map_style=None,
                     tooltip={"text": "Elevation: {elevation} m"})


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
if run_button:
    if not st.session_state.start_latlon or not st.session_state.goal_latlon:
        st.error("Please set both a Start and a Goal point on the map first.")
    else:
        with st.spinner("Computing global path and simulating flight..."):
            result = run_simulation(st.session_state.start_latlon, st.session_state.goal_latlon,
                                     start_alt, goal_alt, cruise_alt)
        if result is None:
            st.error("Global path planner failed to find a valid trajectory between start and goal.")
        else:
            st.session_state.sim_result = result

res = st.session_state.sim_result

if res:
    m = res["metrics"]
    st.subheader("2️⃣ Results")
    c1, c2, c3, c4, c5, c6 = st.columns(6)
    c1.metric("Delivery", "SUCCESS" if m["success"] else "FAILED")
    c2.metric("Flight Distance", f"{m['distance_m']:.1f} m")
    c3.metric("Max Altitude", f"{m['max_alt_m']:.0f} m")
    c4.metric("Energy Consumed", f"{m['energy']:.2f} J")
    c5.metric("Planner Time", f"{m['planner_ms']:.2f} ms")
    c6.metric("Dynamic Re-routes", m["reroutes"])
    if m["reroutes"]:
        st.warning(f"⚠️ The drone re-planned its route {m['reroutes']} time(s) after unexpected obstacles appeared mid-flight (shown as orange markers on the map).")

    tab_map, tab_anim, tab_analytics, tab_compare = st.tabs(
        ["🗺️ 3D Map", "🎬 Live Animation", "📊 Analytics", "📚 Real-World Comparison"]
    )

    with tab_map:
        st.pydeck_chart(make_3d_deck(res), use_container_width=True)
        st.markdown(
            """
            <span class="legend-dot" style="background:#2858DC"></span>Planned A* path&nbsp;&nbsp;
            <span class="legend-dot" style="background:#DC2828"></span>Actual flown path&nbsp;&nbsp;
            <span class="legend-dot" style="background:#FFA500"></span>Re-route event&nbsp;&nbsp;
            <span class="legend-dot" style="background:#A000C8"></span>Moving obstacle&nbsp;&nbsp;
            <span class="legend-dot" style="background:#FF8C00"></span>No-fly zone&nbsp;&nbsp;
            <span class="legend-dot" style="background:#00AA00"></span>Start&nbsp;&nbsp;
            <span class="legend-dot" style="background:#E6B400"></span>Goal
            """,
            unsafe_allow_html=True,
        )
        st.caption("Buildings are colored by height (teal = shorter, orange = taller). Gray drop-lines show altitude at a glance. Drag to rotate/tilt.")

    with tab_anim:
        st.write("Play back the flight step-by-step, including any live re-routing.")
        if st.button("▶️ Play Animation", key="play_anim"):
            placeholder = st.empty()
            info = st.empty()
            actual_geo = res["actual_geo"]
            for idx in range(1, len(actual_geo) + 1):
                placeholder.pydeck_chart(
                    make_3d_deck(res, path_so_far=actual_geo[:idx], drone_pos=actual_geo[idx - 1]),
                    use_container_width=True,
                )
                lat, lon, alt = actual_geo[idx - 1]
                info.caption(f"Step {idx}/{len(actual_geo)} · Altitude: {alt:.0f} m")
                time.sleep(anim_speed)
            st.success("Flight complete.")

    with tab_analytics:
        df = pd.DataFrame(res["trace"])
        a1, a2 = st.columns(2)
        a1.metric("Total Steps", len(df) - 1)
        a2.metric("Avg. Risk Exposure", f"{df['risk'].mean():.2f}")

        st.markdown("**Altitude Profile (climb → cruise → descend)**")
        st.line_chart(df.set_index("step")[["alt_m"]])

        st.markdown("**Cumulative Energy Consumption**")
        st.area_chart(df.set_index("step")[["cum_energy"]])

        st.markdown("**Dynamic Obstacle Risk Exposure Over Time**")
        st.line_chart(df.set_index("step")[["risk"]])

        if res["metrics"]["reroutes"]:
            st.markdown("**Re-route Events**")
            st.dataframe(
                pd.DataFrame({"Re-route #": range(1, res["metrics"]["reroutes"] + 1)}),
                use_container_width=True, hide_index=True,
            )

    with tab_compare:
        st.markdown("""
Real drone-delivery operators combine offline route planning with certified onboard
detect-and-avoid systems; this project simplifies that into a 3D A* macro-planner plus
a potential-field micro-avoidance layer, run over a synthetic (not regulatory) airspace model.

| System | Typical cruise / delivery height | Obstacle handling | Delivery mechanism |
|---|---|---|---|
| **This simulation** | User-set climb → cruise → descend profile | 3D A* global plan + potential-field local avoidance + live re-routing | Direct arrival at goal waypoint |
| **Zipline (Platform 2)** | Cruises around 300 ft (≈91 m), well above rooftops | Pre-flight route planning; a small tethered delivery droid — not the aircraft — descends the last stretch | Tethered droid lowered from altitude with its own camera/sensors |
| **Wing (Alphabet)** | Cruises within FAA Part 107 limits; hovers much lower (≈23 ft / 7 m) to deliver | Onboard sense-and-avoid plus pre-flight airspace checks | Package lowered on a tether while hovering |
| **Amazon Prime Air (MK30)** | Descends to a low hover (≈13 ft / 4 m) to release the package | Onboard Detect-and-Avoid system, no parachute | Package dropped from a low hover |

*Figures for real systems are approximate, publicly reported operating figures and can change as
each program evolves — they're included for educational comparison, not as a regulatory reference.*
        """)

else:
    st.info("Set Start and Goal on the map above, then click **🚀 Run Simulation** in the sidebar.")
