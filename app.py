"""
app.py - AI Delivery Drone Path Planning (Streamlit)

 - Pick Start / Goal by clicking a real map, or jump to a US city preset
 - Climb -> cruise -> descend altitude profile
 - Animated 3D city map (real OpenStreetMap buildings, satellite or street basemap)
 - Dynamic re-routing around obstacles that appear mid-flight
 - Waypoint graph and result graphs
 - Comparison against real drone-delivery systems

Run:  streamlit run app.py
"""
import math

import folium
import pandas as pd
import streamlit as st
import streamlit.components.v1 as components
from streamlit_folium import st_folium

from sim3d import (fetch_osm_buildings, route_bbox, run_simulation,
                   synthetic_buildings)
from viewer import build_viewer_html

st.set_page_config(page_title="Drone Path Planning", page_icon="🚁", layout="wide")

st.markdown(
    """
    <style>
    .hero { padding: 1.1rem 1.4rem; border-radius: 14px;
        background: linear-gradient(135deg, #0f2027 0%, #203a43 50%, #2c5364 100%);
        color: #fff; margin-bottom: 1.1rem; }
    .hero h1 { margin: 0; font-size: 1.6rem; }
    .hero p { margin: 0.3rem 0 0 0; opacity: 0.85; font-size: 0.95rem; }
    </style>
    <div class="hero">
        <h1>🚁 AI Delivery Drone Path Planning</h1>
        <p>Choose a city or click the map to set Start &amp; Goal, then fly an animated 3D route through
        real city buildings. The drone climbs, cruises, descends, and re-plans around obstacles that
        appear mid-flight.</p>
    </div>
    """,
    unsafe_allow_html=True,
)

# Start / Goal pairs roughly 1 km apart.
PRESETS = {
    "New York - Midtown": ((40.74844, -73.98566), (40.75798, -73.98554)),
    "San Francisco - Financial District": ((40.79 - 0.0954, -122.3999), (37.7879, -122.4075)),
    "Chicago - The Loop": ((41.8789, -87.6359), (41.8826, -87.6233)),
    "Hyderabad - HITEC City": ((17.4435, 78.3772), (17.4500, 78.3900)),
    "Custom (click the map)": None,
}
DEFAULT_PRESET = "New York - Midtown"

for key, default in [
    ("start_latlon", PRESETS[DEFAULT_PRESET][0]), ("goal_latlon", PRESETS[DEFAULT_PRESET][1]),
    ("mode", "Start"), ("last_click_id", None), ("sim_result", None), ("preset", DEFAULT_PRESET),
]:
    st.session_state.setdefault(key, default)


def apply_preset():
    pair = PRESETS.get(st.session_state.preset)
    if pair:
        st.session_state.start_latlon, st.session_state.goal_latlon = pair
        st.session_state.sim_result = None


@st.cache_data(ttl=3600, show_spinner=False)
def load_osm(bbox):
    # Raises on failure, so a failed download is never cached.
    return fetch_osm_buildings(bbox)


# ---------------------------------------------------------------------------
# Sidebar controls
# ---------------------------------------------------------------------------
with st.sidebar:
    st.header("✈️ Point Selection")
    st.selectbox("Quick location", list(PRESETS), key="preset", on_change=apply_preset)
    st.radio("Clicking the map sets:", ["Start", "Goal"], key="mode", horizontal=True)
    c1, c2 = st.columns(2)
    if c1.button("Reset Start"):
        st.session_state.start_latlon = None
    if c2.button("Reset Goal"):
        st.session_state.goal_latlon = None

    with st.expander("🛫 Drone Height Profile (climb → cruise → descend)", expanded=True):
        start_alt = st.slider("Start altitude (m)", 0, 200, 10, 5)
        cruise_alt = st.slider("Cruise altitude (m)", 10, 400, 80, 5,
                               help="The drone climbs to this height, flies the long leg at this height, then descends.")
        goal_alt = st.slider("Goal (delivery) altitude (m)", 0, 200, 15, 5)

    with st.expander("🔋 Drone Physics & Wind"):
        base_weight = st.slider("Base weight (kg)", 0.5, 10.0, 3.5, 0.5)
        payload_weight = st.slider("Payload weight (kg)", 0.0, 10.0, 2.0, 0.5)
        speed = st.slider("Cruise speed (m/s)", 3.0, 25.0, 10.0, 1.0)
        battery = st.slider("Battery capacity (Wh)", 50, 600, 200, 10)
        wx = st.slider("Wind X (m/s)", -5.0, 5.0, -2.0, 0.5)
        wy = st.slider("Wind Y (m/s)", -5.0, 5.0, 1.5, 0.5)
        wz = st.slider("Wind Z (m/s)", -5.0, 5.0, 0.0, 0.5)

    with st.expander("🧭 Heuristic Weights"):
        w_dist = st.slider("Distance weight", 0.0, 5.0, 1.0, 0.1)
        w_energy = st.slider("Energy weight", 0.0, 5.0, 1.8, 0.1)
        w_risk = st.slider("Risk weight", 0.0, 5.0, 3.0, 0.1)

    with st.expander("🏢 Obstacles & Dynamic Re-routing", expanded=True):
        add_obstacles = st.checkbox("Add buildings, no-fly zone & moving obstacles", value=True)
        use_osm = st.checkbox("Use real OpenStreetMap buildings", value=True,
                              help="Downloads real building footprints and heights. Falls back to sample buildings if unavailable.")
        inject_surprise = st.checkbox("🚨 Inject an unexpected obstacle mid-flight", value=True,
                                      help="Simulates a new hazard appearing after the flight has started.")
        enable_reroute = st.checkbox("Enable dynamic re-routing", value=True,
                                     help="Turn this off to see what happens when the drone does not react.")
        detect_cells = st.slider("Detection range (cells ahead)", 2, 12, 6, 1,
                                 help="How far ahead the drone checks its route for obstacles.")

    st.divider()
    run_button = st.button("🚀 Run Simulation", type="primary")


# ---------------------------------------------------------------------------
# Step 1: click-to-place map + status
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

st.caption(f"Current click mode: **{st.session_state.mode}**. Change it in the sidebar, then click the map. "
           "Keep the two points under 5 km apart; about 1 km gives the most detailed 3D view.")

s_ll, g_ll = st.session_state.start_latlon, st.session_state.goal_latlon
if s_ll and g_ll:
    center, zoom = [(s_ll[0] + g_ll[0]) / 2, (s_ll[1] + g_ll[1]) / 2], 14
elif s_ll or g_ll:
    center, zoom = list(s_ll or g_ll), 14
else:
    center, zoom = [40.7527, -73.9856], 13

fmap = folium.Map(location=center, zoom_start=zoom, tiles="OpenStreetMap")
folium.TileLayer(
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr="Imagery: Esri, Maxar, Earthstar Geographics", name="Satellite").add_to(fmap)
folium.LayerControl().add_to(fmap)
if s_ll:
    folium.Marker(s_ll, tooltip="Start (Warehouse)", icon=folium.Icon(color="green", icon="play")).add_to(fmap)
if g_ll:
    folium.Marker(g_ll, tooltip="Goal (Customer)", icon=folium.Icon(color="red", icon="flag")).add_to(fmap)
if s_ll and g_ll:
    folium.PolyLine([s_ll, g_ll], color="#2858DC", weight=2, dash_array="6").add_to(fmap)

try:
    map_data = st_folium(fmap, height=420, use_container_width=True, key="picker_map")
except TypeError:  # newer streamlit-folium versions size the map differently
    map_data = st_folium(fmap, height=420, key="picker_map")

if map_data and map_data.get("last_clicked"):
    click = map_data["last_clicked"]
    click_id = (round(click["lat"], 6), round(click["lng"], 6))
    if click_id != st.session_state.last_click_id:
        st.session_state.last_click_id = click_id
        if st.session_state.mode == "Start":
            st.session_state.start_latlon = (click["lat"], click["lng"])
        else:
            st.session_state.goal_latlon = (click["lat"], click["lng"])
        st.session_state.preset = "Custom (click the map)"
        st.rerun()


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
def get_buildings(start, goal):
    """Real OpenStreetMap buildings if possible, otherwise sample buildings."""
    if not use_osm:
        return synthetic_buildings(start, goal), "Sample"
    s, w, n, e = route_bbox(start, goal, 150)
    bbox = (math.floor(s * 1000) / 1000, math.floor(w * 1000) / 1000,
            math.ceil(n * 1000) / 1000, math.ceil(e * 1000) / 1000)
    try:
        found = load_osm(bbox)
        if found:
            return found, "OpenStreetMap"
        st.warning("OpenStreetMap has no buildings here. Using sample buildings instead.")
    except Exception:
        st.warning("Could not download real buildings right now. Using sample buildings instead.")
    return synthetic_buildings(start, goal), "Sample"


if run_button:
    if not st.session_state.start_latlon or not st.session_state.goal_latlon:
        st.error("Please set both a Start and a Goal point on the map first.")
    else:
        start, goal = st.session_state.start_latlon, st.session_state.goal_latlon
        with st.spinner("Loading buildings, planning the route and simulating the flight..."):
            buildings, source = (get_buildings(start, goal) if add_obstacles else ([], "None"))
            cfg = dict(
                start=start, goal=goal, start_alt=start_alt, goal_alt=goal_alt, cruise_alt=cruise_alt,
                base_kg=base_weight, payload_kg=payload_weight, speed=speed, battery_wh=battery,
                wind=(wx, wy, wz), w_dist=w_dist, w_energy=w_energy, w_risk=w_risk,
                add_obstacles=add_obstacles, inject_surprise=inject_surprise,
                reroute=enable_reroute, detect_cells=detect_cells,
                buildings=buildings, source=source,
            )
            try:
                st.session_state.sim_result = run_simulation(cfg)
            except ValueError as error:
                st.session_state.sim_result = None
                st.error(str(error))

res = st.session_state.sim_result

if res:
    m = res["metrics"]
    minutes, seconds = divmod(int(round(m["time_s"])), 60)
    st.subheader("2️⃣ Results")

    r1 = st.columns(6)
    r1[0].metric("Delivery", "SUCCESS" if m["success"] else "FAILED")
    r1[1].metric("Flight Distance", f"{m['distance_m']:.0f} m")
    r1[2].metric("Flight Time", f"{minutes}m {seconds:02d}s")
    r1[3].metric("Energy Used", f"{m['energy_wh']:.1f} Wh")
    r1[4].metric("Battery Left", f"{m['battery_left_pct']:.0f}%")
    r1[5].metric("Max Altitude", f"{m['max_alt_m']:.0f} m")

    r2 = st.columns(6)
    r2[0].metric("Dynamic Re-routes", m["reroutes"])
    r2[1].metric("Holds", m["holds"])
    r2[2].metric("Collisions", m["collisions"])
    r2[3].metric("Closest Approach", "-" if m["min_clearance_m"] is None else f"{m['min_clearance_m']:.0f} m")
    r2[4].metric("Path Efficiency", f"{m['efficiency'] * 100:.0f}%",
                 help="Straight-line distance divided by distance actually flown.")
    r2[5].metric("Planner Time", f"{m['planner_ms']:.0f} ms")

    if m["collisions"]:
        st.error(f"The drone entered an obstacle on {m['collisions']} step(s). "
                 "Turn on dynamic re-routing so it can avoid them.")
    elif m["reroutes"]:
        st.warning(f"⚠️ The drone re-planned its route {m['reroutes']} time(s) after obstacles appeared "
                   "(orange markers on the map).")
    if not m["success"]:
        st.error("The drone did not reach the goal. Try a different cruise altitude or detection range.")

    tab_3d, tab_points, tab_results, tab_compare = st.tabs(
        ["🎬 3D Flight", "📍 Waypoint Graph", "📊 Result Graphs", "📚 Real-World Comparison"])

    with tab_3d:
        components.html(build_viewer_html(res, style="satellite"), height=690)
        st.caption(
            f"Buildings: {res['source']}. Grid cell: {m['cell_m']:.0f} m. "
            "Press Play flight. Drag to rotate, scroll to zoom, right-drag to tilt. "
            "Buildings are colored by height (teal is shorter, orange is taller)."
        )

    planned = pd.DataFrame(res["planned_local"])
    actual = pd.DataFrame(res["actual_local"])
    planned["dist_m"] = [0.0] + list(
        (planned[["east_m", "north_m", "alt_m"]].diff().pow(2).sum(axis=1) ** 0.5).cumsum().iloc[1:])
    actual["dist_m"] = [t["cum_dist_m"] for t in res["trace"]]

    with tab_points:
        st.markdown("**Top-down view of the route points** (metres east and north of the start)")
        pts = pd.concat([planned.assign(Path="Planned waypoints"), actual.assign(Path="Flown path")])
        st.scatter_chart(pts, x="east_m", y="north_m", color="Path")

        st.markdown("**Altitude along the route**")
        prof = pd.concat([planned.assign(Path="Planned"), actual.assign(Path="Flown")])
        st.line_chart(prof, x="dist_m", y="alt_m", color="Path")

        with st.expander("Waypoint table"):
            table = planned.rename(columns={"east_m": "East (m)", "north_m": "North (m)",
                                            "alt_m": "Altitude (m)", "dist_m": "Distance (m)"})
            table.insert(0, "Waypoint", range(len(table)))
            st.dataframe(table, hide_index=True)

    with tab_results:
        df = pd.DataFrame(res["trace"])
        a1, a2, a3 = st.columns(3)
        a1.metric("Total Steps", len(df) - 1)
        a2.metric("Average Risk", f"{df['risk'].mean() * 100:.1f}%")
        a3.metric("Peak Risk", f"{df['risk'].max() * 100:.0f}%")

        st.markdown("**Distance: straight line vs planned vs flown**")
        st.bar_chart(pd.DataFrame(
            {"Metres": [m["straight_m"], m["planned_m"], m["distance_m"]]},
            index=["Straight line", "Planned", "Flown"]))

        st.markdown("**Altitude profile (climb → cruise → descend)**")
        st.line_chart(df.set_index("step")[["alt_m"]])

        st.markdown("**Cumulative energy (Wh)**")
        st.area_chart(df.set_index("step")[["cum_energy_wh"]])

        st.markdown("**Obstacle risk over time**")
        st.line_chart(df.set_index("step")[["risk"]])

        st.download_button("Download flight data (CSV)", df.to_csv(index=False),
                           file_name="flight_data.csv", mime="text/csv")

    with tab_compare:
        st.markdown("""
Real drone-delivery operators combine offline route planning with certified onboard
detect-and-avoid systems; this project simplifies that into a 3D A* planner with live
re-routing, run over a synthetic (not regulatory) airspace model.

| System | Typical cruise / delivery height | Obstacle handling | Delivery mechanism |
|---|---|---|---|
| **This simulation** | User-set climb → cruise → descend profile | 3D A* global plan + live re-routing around detected obstacles | Direct arrival at goal waypoint |
| **Zipline (Platform 2)** | Cruises around 300 ft (≈91 m), well above rooftops | Pre-flight route planning; a small tethered delivery droid — not the aircraft — descends the last stretch | Tethered droid lowered from altitude with its own camera/sensors |
| **Wing (Alphabet)** | Cruises within FAA Part 107 limits; hovers much lower (≈23 ft / 7 m) to deliver | Onboard sense-and-avoid plus pre-flight airspace checks | Package lowered on a tether while hovering |
| **Amazon Prime Air (MK30)** | Descends to a low hover (≈13 ft / 4 m) to release the package | Onboard Detect-and-Avoid system, no parachute | Package dropped from a low hover |

*Figures for real systems are approximate, publicly reported operating figures and can change as
each program evolves — they're included for educational comparison, not as a regulatory reference.*
        """)

else:
    st.info("A city is already selected. Click **🚀 Run Simulation** in the sidebar to fly it, "
            "or click the map to choose your own Start and Goal.")
