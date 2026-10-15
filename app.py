"""app.py - Streamlit UI for the AI-Based Delivery Drone Path Planning System."""
from __future__ import annotations

import folium
import pandas as pd
import streamlit as st
import streamlit.components.v1 as components
from branca.element import Element
from folium.plugins import Fullscreen
import math
import plotly.graph_objects as go

import sim3d
import viewer

st.set_page_config(page_title="AI Delivery Drone Path Planner", page_icon="🚁", layout="wide")

st.markdown("""
<style>
@import url('https://fonts.googleapis.com/css2?family=Bricolage+Grotesque:wght@600;700&family=Instrument+Sans:wght@400;500;600&display=swap');
html, body, [class*="st-"], button, input { font-family: 'Instrument Sans', system-ui, sans-serif; }
h1, h2, h3 { font-family: 'Bricolage Grotesque', 'Instrument Sans', system-ui, sans-serif; letter-spacing: -0.01em; color: #0f2a43; }
.block-container { padding-top: 2rem; max-width: 1280px; }
[data-testid="stSidebar"] { border-right: 1px solid #cfd9e3; }
[data-testid="stSidebar"] h2 { font-size: 1.15rem; }
/* flight strip: the one memorable element */
.strip { display: grid; grid-template-columns: 1fr minmax(120px, 2fr) 1fr auto; align-items: center; gap: 18px;
  background: #fff; border: 1px solid #cfd9e3; border-left: 6px solid #1d4ed8; border-radius: 6px; padding: 14px 18px; margin: 4px 0 18px; }
.strip .pt b { display: block; font: 600 1.05rem 'Bricolage Grotesque', sans-serif; }
.strip .pt span { color: #52687d; font-size: .82rem; font-variant-numeric: tabular-nums; }
.strip .pt.end { text-align: right; }
.strip .line { position: relative; height: 2px; background: repeating-linear-gradient(90deg, #8aa0b5 0 8px, transparent 8px 14px); text-align: center; }
.strip .line i { position: absolute; left: 50%; top: -11px; transform: translateX(-50%); background: #fff; padding: 0 10px;
  font-style: normal; font-weight: 600; font-size: .85rem; font-variant-numeric: tabular-nums; }
.strip .line::before, .strip .line::after { content: ""; position: absolute; top: -4px; width: 10px; height: 10px; border-radius: 50%; }
.strip .line::before { left: -4px; background: #16a34a; } .strip .line::after { right: -4px; background: #dc2626; }
.chip { padding: 5px 12px; border-radius: 999px; font-weight: 600; font-size: .85rem; white-space: nowrap; }
.chip.idle { background: #e6ecf2; color: #52687d; } .chip.good { background: #dcfce7; color: #166534; }
.chip.warn { background: #fef3c7; color: #92400e; } .chip.bad { background: #fee2e2; color: #991b1b; }
@media (max-width: 760px) { .strip { grid-template-columns: 1fr; } .strip .pt.end { text-align: left; } .strip .line { margin: 14px 0; } }
/* quiet supporting chrome */
[data-testid="stMetric"] { background: #fff; border: 1px solid #d9e2ea; border-radius: 6px; padding: 10px 14px; }
[data-testid="stMetricLabel"] p { color: #52687d; font-size: .82rem; }
[data-testid="stMetricValue"] { font-variant-numeric: tabular-nums; font-weight: 600; }
.stTabs [data-baseweb="tab-list"] { gap: 4px; border-bottom: 1px solid #cfd9e3; }
.stTabs [data-baseweb="tab"] { height: 44px; padding: 0 16px; font-weight: 600; }
.stTabs [aria-selected="true"] { color: #1d4ed8; }
.stButton > button[kind="primary"] { font-weight: 600; border-radius: 6px; }
[data-testid="stDataFrame"] { border: 1px solid #d9e2ea; border-radius: 6px; }
</style>
""", unsafe_allow_html=True)

DEFAULT_START = (17.4435, 78.3772)
DEFAULT_DEST = (17.4401, 78.3911)
PHASE_COLOR = {"takeoff": "#16a34a", "climbing": "#0891b2", "cruising": "#2563eb", "descending": "#0284c7",
               "landing": "#16a34a"}


def stretch(widget, *args, **kwargs):
    """Full-width widget on old and new Streamlit (use_container_width was replaced by width="stretch")."""
    try:
        return widget(*args, width="stretch", **kwargs)
    except Exception:
        return widget(*args, use_container_width=True, **kwargs)


def m(v: float) -> str:
    """One distance format everywhere: metres with one decimal."""
    return f"{v:,.1f} m"


# ---------------------------------------------------------------- state
ss = st.session_state
ss.setdefault("start", DEFAULT_START)
ss.setdefault("dest", DEFAULT_DEST)
ss.setdefault("result", None)
ss.setdefault("last_click", None)
ss.setdefault("step", 0)
ss.setdefault("pick_n", 0)
ss.setdefault("pick_center", None)
if "pick_next" in ss:                       # set before the radio is created (Streamlit rule)
    ss["pick_mode"] = ss.pop("pick_next")
ss.setdefault("pick_mode", "START")

# ---------------------------------------------------------------- sidebar
with st.sidebar:
    st.header("Mission settings")
    altitude = st.slider("Cruise altitude (m above ground)", 10, 150, 60, 5)
    ceiling = st.slider("Planning ceiling (m above ground)", 30, 150, max(120, altitude), 5)
    c1, c2 = st.columns(2)
    mass = c1.number_input("Drone mass (kg)", 0.5, 25.0, 2.0, 0.1)
    payload = c2.number_input("Payload (kg)", 0.0, 10.0, 0.5, 0.1)
    speed = st.slider("Ground speed (m/s)", 3, 25, 12)
    st.caption("Wind = direction the air moves TOWARD (m/s).")
    w1, w2 = st.columns(2)
    wind_e = w1.number_input("Wind east (+) / west (−)", -15.0, 15.0, 0.0, 0.5)
    wind_n = w2.number_input("Wind north (+) / south (−)", -15.0, 15.0, 0.0, 0.5)
    battery = st.number_input("Battery capacity (Wh)", 10.0, 1000.0, 80.0, 5.0)
    st.subheader("Path-cost weights")
    w_dist = st.slider("Distance", 0.1, 5.0, 1.0, 0.1)
    w_climb = st.slider("Climbing (per metre of altitude change)", 0.0, 5.0, 0.5, 0.1)
    w_risk = st.slider("Risk (closeness to obstacles)", 0.0, 10.0, 2.0, 0.5)
    st.subheader("Obstacles")
    detect = st.slider("Detection range (m)", 20, 300, 80, 10)
    margin = st.slider("Safety margin (m)", 2, 30, 8)
    use_osm = st.toggle("Use real buildings (OpenStreetMap)", True,
                        help="Falls back to synthetic buildings if the lookup fails.")
    use_nofly = st.toggle("No-fly zones", True)
    use_dyn = st.toggle("Moving obstacles + dynamic re-routing", True)
    response = st.selectbox("When a moving obstacle threatens the route",
                            ["auto", "reroute", "hold"],
                            format_func={"auto": "Auto (re-route, or hold if cheaper)", "reroute": "Prefer re-route",
                                         "hold": "Prefer hold (wait for it to pass)"}.get)
    run = stretch(st.button, "▶ Run simulation", type="primary")

# ---------------------------------------------------------------- run
if run:
    cfg = sim3d.Config(
        start=tuple(ss["start"]), dest=tuple(ss["dest"]), altitude_m=float(altitude), ceiling_m=float(ceiling),
        drone_mass_kg=float(mass), payload_kg=float(payload), speed_ms=float(speed), wind_east_ms=float(wind_e),
        wind_north_ms=float(wind_n), battery_wh=float(battery), w_dist=float(w_dist), w_climb=float(w_climb),
        w_risk=float(w_risk), detect_range_m=float(detect), safety_margin_m=float(margin), use_osm=use_osm,
        use_nofly=use_nofly, use_dynamic=use_dyn, response=response)
    with st.spinner("Planning and flying the mission…"):
        ss["result"] = sim3d.run_simulation(cfg)
    ss["step"] = 0

st.title("AI delivery drone path planner")
res = ss["result"]


def flight_strip(res):
    """Start, destination, distance and mission status at a glance."""
    d = sim3d.haversine_m(*ss["start"], *ss["dest"])
    if res is None:
        cls, label = "idle", "Ready to run"
    elif not res.get("ok"):
        cls, label = "bad", "Could not run"
    elif res["failure"]:
        cls, label = "bad", "Not completed"
    elif res["metrics"]["success"]:
        cls, label = "good", "Mission completed"
    else:
        cls, label = "warn", "Touched an obstacle"
    mid = f"{m(d)} straight"
    if res and res.get("ok"):
        mid = f"{m(res['metrics']['actual_distance_m'])} flown"
    st.markdown(
        f'<div class="strip"><div class="pt"><b>Start</b><span>{ss["start"][0]:.5f}, {ss["start"][1]:.5f}</span></div>'
        f'<div class="line"><i>{mid}</i></div>'
        f'<div class="pt end"><b>Destination</b><span>{ss["dest"][0]:.5f}, {ss["dest"][1]:.5f}</span></div>'
        f'<span class="chip {cls}">{label}</span></div>', unsafe_allow_html=True)



def pick_figure():
    """Pure-Plotly click map (no custom component needed). Returns (figure, grid_lats, grid_lons)."""
    if ss.get("pick_center") is None:
        ss["pick_center"] = ((ss["start"][0] + ss["dest"][0]) / 2, (ss["start"][1] + ss["dest"][1]) / 2)
    clat, clon = ss["pick_center"]
    ext = max(abs(ss["start"][0] - ss["dest"][0]) * 111320,
              abs(ss["start"][1] - ss["dest"][1]) * 111320 * math.cos(math.radians(clat)), 800)
    half_m = ext * 1.2
    n = 61
    dlat = half_m / 111320
    dlon = half_m / (111320 * math.cos(math.radians(clat)))
    grid_lat, grid_lon = [], []
    for i in range(n):
        for j in range(n):
            grid_lat.append(clat - dlat + 2 * dlat * i / (n - 1))
            grid_lon.append(clon - dlon + 2 * dlon * j / (n - 1))
    zoom = max(1, min(18, math.log2(156543 * math.cos(math.radians(clat)) * 900 / (2 * half_m))))
    new = hasattr(go, "Scattermap")
    Sc = go.Scattermap if new else go.Scattermapbox
    fig = go.Figure()
    fig.add_trace(Sc(lat=grid_lat, lon=grid_lon, mode="markers", marker=dict(size=16, opacity=0.02, color="#1d4ed8"),
                     hoverinfo="skip", name="grid"))
    fig.add_trace(Sc(lat=[ss["start"][0], ss["dest"][0]], lon=[ss["start"][1], ss["dest"][1]], mode="lines",
                     line=dict(width=2, color="#666"), hoverinfo="skip", name="line"))
    fig.add_trace(Sc(lat=[ss["start"][0]], lon=[ss["start"][1]], mode="markers+text", text=["START"],
                     textposition="top right", marker=dict(size=16, color="#16a34a"), hoverinfo="skip", name="START"))
    fig.add_trace(Sc(lat=[ss["dest"][0]], lon=[ss["dest"][1]], mode="markers+text", text=["DESTINATION"],
                     textposition="top right", marker=dict(size=16, color="#dc2626"), hoverinfo="skip", name="DEST"))
    layout = dict(style="open-street-map", center=dict(lat=clat, lon=clon), zoom=zoom)
    fig.update_layout(**({"map": layout} if new else {"mapbox": layout}), margin=dict(l=0, r=0, t=0, b=0),
                      height=480, showlegend=False, clickmode="event+select", dragmode="pan")
    return fig, grid_lat, grid_lon

flight_strip(res)

tab_pick, tab_map, tab_3d, tab_tables = st.tabs(
    ["Pick route", "Route map and steps", "3D viewer", "Tables and checks"])

# ---------------------------------------------------------------- tab 1: pick
with tab_pick:
    st.write("Click the map to place the points. Choose which one the **next click** sets, then press "
             "**Run simulation** in the sidebar.")
    st.radio("Next click sets", ["START", "DESTINATION"], horizontal=True, key="pick_mode")
    out = None
    fig, grid_lat, grid_lon = pick_figure()
    ev = st.plotly_chart(fig, use_container_width=True, key=f"pickplot{ss['pick_n']}",
                         on_select="rerun", selection_mode="points", config={"scrollZoom": True, "displaylogo": False})
    pts = ((ev or {}).get("selection") or {}).get("points") or []
    hit = [p for p in pts if p.get("curve_number", 0) == 0]
    if hit:
        i = hit[-1].get("point_index", hit[-1].get("point_number"))
        lat, lon = float(grid_lat[i]), float(grid_lon[i])
        if ss["pick_mode"] == "START":
            ss["start"] = (round(lat, 6), round(lon, 6))
            ss["pick_next"] = "DESTINATION"
        else:
            ss["dest"] = (round(lat, 6), round(lon, 6))
        ss["result"] = None
        ss["pick_n"] += 1
        st.rerun()
    c1, c2 = st.columns([1, 3])
    if c1.button("Re-centre map on route"):
        ss["pick_center"] = None
        ss["pick_n"] += 1
        st.rerun()
    c2.caption("Tip: scroll to zoom, drag to pan, click anywhere to place the point (snaps to a ~50-100 m grid; "
               "use the coordinate boxes below for exact values).")
    d_now = sim3d.haversine_m(*ss["start"], *ss["dest"])
    st.caption(f"START {ss['start'][0]:.6f}, {ss['start'][1]:.6f}  ·  DESTINATION {ss['dest'][0]:.6f}, "
               f"{ss['dest'][1]:.6f}  ·  straight-line ≈ {m(d_now)}")
    with st.expander("Or type coordinates"):
        a1, a2, a3, a4 = st.columns(4)
        s_lat = a1.number_input("Start lat", value=float(ss["start"][0]), format="%.6f")
        s_lon = a2.number_input("Start lon", value=float(ss["start"][1]), format="%.6f")
        d_lat = a3.number_input("Dest lat", value=float(ss["dest"][0]), format="%.6f")
        d_lon = a4.number_input("Dest lon", value=float(ss["dest"][1]), format="%.6f")
        if st.button("Apply coordinates"):
            ss["start"], ss["dest"], ss["result"] = (s_lat, s_lon), (d_lat, d_lon), None
            ss["pick_center"] = None
            ss["pick_n"] += 1
            st.rerun()

# ---------------------------------------------------------------- no / failed result
if res is None:
    for t in (tab_map, tab_3d, tab_tables):
        with t:
            st.info("Choose start and destination in Pick route, then press Run simulation in the sidebar.")
    st.stop()
if not res.get("ok"):
    st.error(f"**Simulation could not run.** {res['error']}")
    for t in (tab_map, tab_3d, tab_tables):
        with t:
            st.info("No result to show - see the error above.")
    st.stop()

mt, frames, events = res["metrics"], res["frames"], res["events"]

if res["failure"]:
    st.error(f"**Mission not completed:** {res['failure']}")
elif mt["success"]:
    st.success("Mission completed: the drone reached the destination without contacting any obstacle.")
else:
    st.warning("The drone arrived but came into contact with an obstacle (clearance 0 m). See the tables.")
for w in res.get("warnings", []):
    st.warning(w)
if res["buildings_source"] != "OpenStreetMap" and not res.get("warnings"):
    st.info(res["buildings_note"])

# ---------------------------------------------------------------- helpers for the 2D map
def leg_popup(r, l):
    w0, w1 = r["waypoints"][l["a"]], r["waypoints"][l["b"]]
    clr = (f"{l['clearance_obj']} at {m(l['min_clearance_m'])}" if l["clearance_obj"] else "none within 100 m")
    return (f"<b>Route v{r['ver']} · leg {l['idx'] + 1}/{len(r['legs'])} · {sim3d.PHASE_NAME[l['phase']]}</b><br>"
            f"From {w0['label']} ({l['lat0']:.6f}, {l['lon0']:.6f}) at {m(l['z0'])} AGL<br>"
            f"To {w1['label']} ({l['lat1']:.6f}, {l['lon1']:.6f}) at {m(l['z1'])} AGL<br>"
            f"Segment length: <b>{m(l['length_m'])}</b> (horizontal {m(l['horiz_m'])}, vertical {l['vert_m']:+.1f} m)<br>"
            f"Cumulative from takeoff: {m(l['cum_start_takeoff_m'])} → <b>{m(l['cum_end_takeoff_m'])}</b><br>"
            f"Closest obstacle: {clr}<br><i>{l['why']}</i>")


def wp_popup(r, w):
    clr = f"{w['clearance_obj']} at {m(w['clearance_m'])}" if w["clearance_obj"] else "none within 100 m"
    return (f"<b>{w['label']}</b> (route v{r['ver']})<br>Lat/Lon: {w['lat']:.6f}, {w['lon']:.6f}<br>"
            f"Local x, y: {w['x']:.1f}, {w['y']:.1f} m<br>Altitude: {m(w['z'])} AGL<br>"
            f"Cumulative from takeoff: <b>{m(w['cum_takeoff_m'])}</b><br>Closest obstacle: {clr}<br><i>{w['why']}</i>")


LEGEND = """
<div style="position:fixed;bottom:24px;left:12px;z-index:9999;background:white;border:1px solid #999;border-radius:8px;
 padding:8px 10px;font:12px/1.5 sans-serif;max-width:280px;box-shadow:0 1px 4px #0004">
 <b>Legend</b><br>
 <span style="color:#2563eb">━━</span> planned route (click a leg)<br>
 <span style="color:#16a34a">━━</span> actual flown path<br>
 <span style="color:#ea580c">━━</span> re-route<br>
 <span style="color:#888">╌╌</span> straight line start → destination<br>
 <span style="color:#ea580c">●</span> re-route event &nbsp;<span style="color:#7c3aed">●</span> hold event<br>
 <span style="color:#dc2626">●</span> no-fly zone / moving obstacle &nbsp;<span style="color:#eab308">●</span> drone at step<br>
 <hr style="margin:4px 0">
 <b>Planned</b> = 3D route length · <b>Straight-line</b> = ground distance, ignoring obstacles ·
 <b>Actual</b> = distance really flown. All in metres.
</div>"""


def result_map(frame_idx: int):
    cam, meta = res["camera"], res["meta"]
    mm = folium.Map(location=[cam["lat"], cam["lon"]], zoom_start=int(cam["zoom"]), tiles="OpenStreetMap")
    sw, ne = meta["bounds_geo"]["sw"], meta["bounds_geo"]["ne"]
    mm.fit_bounds([sw, ne])
    fg = {k: folium.FeatureGroup(name=k, show=v) for k, v in (
        ("Buildings", True), ("No-fly zones", True), ("Planned route", True), ("Re-routes", True),
        ("Actual path", True), ("Waypoints", True), ("Events", True), ("Moving obstacle traces", True),
        ("Drone (selected step)", True))}
    for b in res["buildings"][:600]:
        folium.Polygon([[p[1], p[0]] for p in b["polygon"]], color="#64748b", weight=1, fill=True,
                       fill_opacity=0.35, tooltip=f"{b['name']} · {b['src']}").add_to(fg["Buildings"])
    for z in res["nofly"]:
        folium.Circle([z["lat"], z["lon"]], radius=z["radius"], color="#dc2626", fill=True, fill_opacity=0.25,
                      tooltip=f"{z['name']} · radius {m(z['radius'])}",
                      popup=f"<b>{z['name']}</b><br>Radius {m(z['radius'])}<br>Closed from ground to {m(z['top'])} AGL"
                      ).add_to(fg["No-fly zones"])
    for ob in res["traces"]["obstacles"]:
        folium.PolyLine([[p[1], p[0]] for p in ob["path"]], color="#dc2626", weight=2, dash_array="2 6",
                        tooltip=f"{ob['name']} trajectory").add_to(fg["Moving obstacle traces"])
    folium.PolyLine([[res["start"]["lat"], res["start"]["lon"]], [res["dest"]["lat"], res["dest"]["lon"]]],
                    color="#888", weight=2, dash_array="8", tooltip=f"Straight line: {m(mt['straight_line_m'])}"
                    ).add_to(mm)
    for r in res["routes"]:
        group = fg["Planned route"] if r["ver"] == 0 else fg["Re-routes"]
        color = "#2563eb" if r["ver"] == 0 else "#ea580c"
        for l in r["legs"]:
            folium.PolyLine([[l["lat0"], l["lon0"]], [l["lat1"], l["lon1"]]], color=color, weight=5, opacity=0.85,
                            tooltip=(f"v{r['ver']} leg {l['idx'] + 1} · {sim3d.PHASE_NAME[l['phase']]} · "
                                     f"{m(l['length_m'])} · cum. {m(l['cum_end_takeoff_m'])}"),
                            popup=folium.Popup(leg_popup(r, l), max_width=360)).add_to(group)
        for w in r["waypoints"]:
            if r["ver"] > 0 and w["kind"] == "destination":
                continue                       # the destination marker is already drawn by route v0
            col = {"start": "#16a34a", "destination": "#dc2626"}.get(w["kind"], color)
            folium.CircleMarker([w["lat"], w["lon"]], radius=6, color="#fff", weight=1, fill=True, fill_color=col,
                                fill_opacity=1, tooltip=f"{w['label']} · {m(w['z'])} AGL · {m(w['cum_takeoff_m'])} from takeoff",
                                popup=folium.Popup(wp_popup(r, w), max_width=340)).add_to(fg["Waypoints"])
    folium.PolyLine([[f["lat"], f["lon"]] for f in frames], color="#16a34a", weight=3, opacity=0.9,
                    tooltip=f"Actual flown path: {m(mt['actual_distance_m'])}").add_to(fg["Actual path"])
    for e in events:
        col = {"reroute": "orange", "hold": "purple", "resume": "green", "abort": "red"}.get(e["type"], "gray")
        icon = {"reroute": "random", "hold": "pause", "resume": "play", "abort": "remove"}.get(e["type"], "info-sign")
        folium.Marker([e["lat"], e["lon"]], icon=folium.Icon(color=col, icon=icon),
                      tooltip=f"{e['type'].upper()} · t={e['t']:.0f} s · {e['detail']}",
                      popup=folium.Popup(f"<b>{e['type'].upper()}</b> at t = {e['t']:.1f} s<br>"
                                         f"Location {e['lat']:.6f}, {e['lon']:.6f} · {m(e['alt_m'])} AGL<br>"
                                         f"Flown so far: {m(e['cum_m'])}<br>{e['reason']}<br><i>{e['detail']}</i>",
                                         max_width=360)).add_to(fg["Events"])
    f = frames[frame_idx]
    folium.CircleMarker([f["lat"], f["lon"]], radius=9, color="#000", weight=2, fill=True, fill_color="#eab308",
                        fill_opacity=1, tooltip=f"Drone at t={f['t']:.1f} s · {m(f['z'])} AGL").add_to(fg["Drone (selected step)"])
    for ob in res["dynamic"]:
        p = (ob["p0"][0] + ob["v"][0] * f["t"], ob["p0"][1] + ob["v"][1] * f["t"])
        la, lo = sim3d.GeoFrame(meta["origin_lat"], meta["origin_lon"]).to_geo(*p)
        folium.Circle([la, lo], radius=ob["radius"], color="#dc2626", fill=True, fill_opacity=0.5,
                      tooltip=f"{ob['name']} at t={f['t']:.1f} s").add_to(fg["Drone (selected step)"])
    for g in fg.values():
        g.add_to(mm)
    folium.LayerControl(collapsed=True).add_to(mm)
    Fullscreen().add_to(mm)
    mm.get_root().html.add_child(Element(LEGEND))
    return mm


# ---------------------------------------------------------------- tab 2: map + inspector
with tab_map:
    k1, k2, k3, k4 = st.columns(4)
    k1.metric("Planned route (3D)", m(mt["planned_distance_m"]))
    k2.metric("Straight-line (ground)", m(mt["straight_line_m"]))
    k3.metric("Actual flown", m(mt["actual_distance_m"]), f"{mt['actual_minus_planned_m']:+.1f} m vs planned",
              delta_color="off")
    k4.metric("Detour factor", f"{mt['detour_factor']:.3f}×")
    st.info("**Planned** is the length of the 3D route the planner chose (climb + cruise + descent + detours). "
            "**Straight-line** is the shortest ground distance start → destination, ignoring obstacles. "
            "**Actual** is the distance really flown: it equals planned unless a re-route happened (a hold adds "
            "time and energy, not distance). Click any leg or waypoint on the map for its coordinates, altitude, "
            "segment length, cumulative distance and closest obstacle.")
    st.slider("Step inspector (simulation step)", 0, len(frames) - 1, key="step",
              help="Moves the drone marker and updates the explanation below. The 3D viewer has its own "
                   "play / pause / seek controls.")
    fr = frames[ss["step"]]
    components.html(result_map(ss["step"]).get_root().render(), height=580, scrolling=False)
    ev = next((e for e in events if e["id"] == fr["ev"]), None)
    st.markdown(f"**Step {ss['step']} · t = {fr['t']:.1f} s · {sim3d.PHASE_NAME[fr['state']]}** — {fr['text']}")
    q1, q2, q3, q4, q5 = st.columns(5)
    q1.metric("Altitude (AGL)", m(fr["z"]))
    q2.metric("Travelled", m(fr["dist"]))
    q3.metric("Remaining", m(fr["rem"]))
    q4.metric("Energy used", f"{fr['e_used']:.2f} Wh", f"{fr['e_left']:.2f} Wh left ({fr['batt']:.0f}%)", delta_color="off")
    q5.metric("Clearance", m(fr["clr"]), f"{fr['risk']}" + (f" · {fr['clr_obj']}" if fr["clr_obj"] else ""),
              delta_color="off")
    st.caption(f"Position {fr['lat']:.6f}, {fr['lon']:.6f} · route v{fr['ver']}, leg {fr['leg'] + 1}")
    if ev:
        st.warning(f"**{ev['type'].upper()}** — {ev['reason']} {ev['detail']}")

# ---------------------------------------------------------------- tab 3: 3D viewer
with tab_3d:
    st.caption("Playback, seek, speed, follow and click-to-inspect are inside the viewer. "
               "The map libraries load from unpkg.com, so an internet connection is required for the 3D map.")
    components.html(viewer.build_viewer_html(res), height=780, scrolling=False)

# ---------------------------------------------------------------- tab 4: tables
with tab_tables:
    st.subheader("Mission metrics")
    cols = st.columns(4)
    items = [
        ("Success", "yes" if mt["success"] else "no"), ("Flight time", f"{mt['flight_time_s']:.1f} s"),
        ("Planned time", f"{mt['planned_time_s']:.1f} s"), ("Hold time", f"{mt['hold_time_s']:.1f} s"),
        ("Energy used", f"{mt['energy_used_wh']:.2f} Wh"), ("Planned energy", f"{mt['planned_energy_wh']:.2f} Wh"),
        ("Battery left", f"{mt['battery_left_wh']:.1f} Wh ({mt['battery_left_pct']:.0f}%)"),
        ("Max altitude", m(mt["max_altitude_m"])), ("Min clearance", m(mt["min_clearance_m"])),
        ("Re-routes", mt["reroute_count"]), ("Holds", mt["hold_count"]), ("Avg ground speed", f"{mt['avg_ground_speed_ms']:.1f} m/s"),
        ("Buildings", f"{mt['building_count']} ({res['buildings_source']})"), ("No-fly zones", mt["nofly_count"]),
        ("Moving obstacles", mt["dynamic_count"]), ("A* nodes expanded", f"{mt['nodes_expanded']:,}"),
    ]
    for i, (k, v) in enumerate(items):
        cols[i % 4].metric(k, v)

    st.subheader("Planned path (route v0): waypoints")
    r0 = res["routes"][0]
    stretch(st.dataframe, pd.DataFrame([{
        "Waypoint": w["label"], "Latitude": round(w["lat"], 6), "Longitude": round(w["lon"], 6),
        "Altitude AGL (m)": round(w["z"], 1), "Cumulative from takeoff (m)": round(w["cum_takeoff_m"], 1),
        "Closest obstacle": (f"{w['clearance_obj']} ({w['clearance_m']:.1f} m)" if w["clearance_obj"] else "-"),
        "Why": w["why"]} for w in r0["waypoints"]]), hide_index=True)

    st.subheader("Route legs (all route versions)")
    stretch(st.dataframe, pd.DataFrame([{
        "Route": f"v{r['ver']}", "Leg": l["idx"] + 1, "Phase": sim3d.PHASE_NAME[l["phase"]],
        "Horizontal (m)": round(l["horiz_m"], 1), "Vertical (m)": round(l["vert_m"], 1),
        "Length 3D (m)": round(l["length_m"], 1), "Cumulative start (m)": round(l["cum_start_takeoff_m"], 1),
        "Cumulative end (m)": round(l["cum_end_takeoff_m"], 1), "Time (s)": round(l["time_s"], 1),
        "Energy (Wh)": round(l["energy_wh"], 2), "Min clearance (m)": round(l["min_clearance_m"], 1),
        "Closest obstacle": l["clearance_obj"] or "-"} for r in res["routes"] for l in r["legs"]]),
        hide_index=True)

    st.subheader("Events (re-routes and holds)")
    if events:
        stretch(st.dataframe, pd.DataFrame([{
            "Event": e["id"], "Type": e["type"], "t (s)": round(e["t"], 1), "Latitude": round(e["lat"], 6),
            "Longitude": round(e["lon"], 6), "Altitude (m)": round(e["alt_m"], 1), "Flown so far (m)": round(e["cum_m"], 1),
            "Reason": e["reason"], "Distance impact": e["detail"]} for e in events]),
            hide_index=True)
    else:
        st.caption("No re-route or hold events in this flight.")

    st.subheader("Actual flown path (one row per simulation frame)")
    stretch(st.dataframe, pd.DataFrame([{
        "Step": i, "t (s)": round(f["t"], 1), "State": sim3d.PHASE_NAME[f["state"]], "Latitude": round(f["lat"], 6),
        "Longitude": round(f["lon"], 6), "Altitude AGL (m)": round(f["z"], 1), "Travelled (m)": round(f["dist"], 1),
        "Remaining (m)": round(f["rem"], 1), "Energy used (Wh)": round(f["e_used"], 2),
        "Clearance (m)": round(f["clr"], 1), "Risk": f["risk"]} for i, f in enumerate(frames)]),
        hide_index=True)

    st.subheader("Consistency checks")
    issues = res.get("validation", [])
    if issues:
        for i in issues:
            st.error(i)
    else:
        st.success("Coordinate round-trips, distance totals, frame-to-frame distances and on-route interpolation all "
                   "passed.")
    st.caption("Terrain is assumed flat: altitude and building heights are both metres above ground level. "
               "Planning is treated as instantaneous (no time passes while a re-route is computed).")
