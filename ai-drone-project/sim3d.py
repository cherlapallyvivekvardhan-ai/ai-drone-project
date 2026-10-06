"""sim3d.py - AI-Based Delivery Drone Path Planning: simulation engine.

Standard library only (no Streamlit). Everything here uses ONE coordinate
reference, so the map, the 2D route and the 3D viewer can never disagree:

    Local frame (ENU, metres), origin = the START point
        x = metres EAST  of the origin      (lon -> x)
        y = metres NORTH of the origin      (lat -> y)
        z = metres ABOVE GROUND LEVEL (AGL). Terrain is assumed flat, so a
            building's height and the drone's z are measured from the same
            ground (z = 0).
    Geographic <-> local conversion is the linear (equirectangular) mapping
        x = (lon - lon0) * m_per_deg_lon,  y = (lat - lat0) * m_per_deg_lat
    and its exact inverse, using the metres-per-degree values at the origin
    latitude (WGS-84 series). These two numbers are also shipped to the viewer
    (result["meta"]) so the browser uses the identical mapping.

Distances (all metres, all computed in the local frame):
    planned distance   = 3D length of the planned route (climb + cruise + descent)
    straight-line      = ground distance start -> destination (both at z = 0)
    actual distance    = 3D distance really flown (includes re-routes; a hold adds
                         flight time and energy but no distance)

Run `python sim3d.py --selftest` to execute the built-in consistency checks.
"""
from __future__ import annotations

import heapq
import itertools
import json
import math
import random
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import asdict, dataclass

G = 9.80665          # m/s^2
RHO = 1.225          # kg/m^3 air density
CLEAR_CAP = 100.0    # clearances are reported up to this distance (m)
ROTOR_AREA = 0.30    # total rotor disk area (m^2)
ETA_PROP = 0.65      # propulsive efficiency
CD_A = 0.08          # drag area (m^2)
P_AVIONICS = 15.0    # W
HASH = 60.0          # spatial hash cell (m)

PHASE_NAME = {
    "takeoff": "Takeoff", "climbing": "Climbing", "cruising": "Cruising",
    "descending": "Descending", "landing": "Landing", "holding": "Holding",
    "rerouting": "Re-routing", "landed": "Landed", "aborted": "Aborted",
}


class SimError(Exception):
    """A readable, user-facing problem (bad input, no route, ...)."""


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------
@dataclass
class Config:
    start: tuple = (17.4435, 78.3772)     # (lat, lon)
    dest: tuple = (17.4401, 78.3911)
    altitude_m: float = 60.0              # cruise altitude, metres AGL
    ceiling_m: float = 120.0              # highest altitude the planner may use (AGL)
    drone_mass_kg: float = 2.0
    payload_kg: float = 0.5
    speed_ms: float = 12.0                # ground speed on level legs
    wind_east_ms: float = 0.0             # direction the air moves TOWARD
    wind_north_ms: float = 0.0
    battery_wh: float = 80.0
    reserve_pct: float = 20.0
    w_dist: float = 1.0                   # path-cost weights
    w_climb: float = 0.5
    w_risk: float = 2.0
    detect_range_m: float = 80.0          # dynamic obstacle detection range
    safety_margin_m: float = 8.0
    vz_max_ms: float = 4.0                # max climb/descent rate
    use_osm: bool = True                  # try OpenStreetMap buildings first
    use_nofly: bool = True
    use_dynamic: bool = True
    response: str = "auto"                # auto | reroute | hold
    max_hold_s: float = 30.0
    seed: int | None = None
    overpass_timeout_s: float = 10.0


# --------------------------------------------------------------------------
# Geometry / coordinate conversion
# --------------------------------------------------------------------------
class GeoFrame:
    """Local ENU frame anchored at (lat0, lon0)."""

    def __init__(self, lat0: float, lon0: float):
        self.lat0, self.lon0 = lat0, lon0
        p = math.radians(lat0)
        self.m_per_deg_lat = (111132.92 - 559.82 * math.cos(2 * p)
                              + 1.175 * math.cos(4 * p) - 0.0023 * math.cos(6 * p))
        self.m_per_deg_lon = (111412.84 * math.cos(p) - 93.5 * math.cos(3 * p)
                              + 0.118 * math.cos(5 * p))

    def to_local(self, lat: float, lon: float):
        return ((lon - self.lon0) * self.m_per_deg_lon, (lat - self.lat0) * self.m_per_deg_lat)

    def to_geo(self, x: float, y: float):
        return (self.lat0 + y / self.m_per_deg_lat, self.lon0 + x / self.m_per_deg_lon)

    def meta(self):
        return {"origin_lat": self.lat0, "origin_lon": self.lon0,
                "m_per_deg_lat": self.m_per_deg_lat, "m_per_deg_lon": self.m_per_deg_lon}


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((p2 - p1) / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * r * math.asin(math.sqrt(a))


def dist3(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def poly_dist(x, y, poly):
    """Distance from a point to a polygon (0 if inside)."""
    inside, best, n = False, 1e18, len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        dx, dy = xj - xi, yj - yi
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - xi) * dx + (y - yi) * dy) / l2))
        d = math.hypot(x - (xi + t * dx), y - (yi + t * dy))
        if d < best:
            best = d
        j = i
    return 0.0 if inside else best


def bearing_deg(dx, dy):
    """Compass bearing (0 = north, 90 = east) of a local (east, north) vector."""
    return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0


# --------------------------------------------------------------------------
# Buildings: OpenStreetMap (Overpass) with synthetic fallback
# --------------------------------------------------------------------------
def _make_building(bid, poly, height, assumed, src):
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    return {"id": bid, "name": f"Building {bid} ({height:.0f} m{'~' if assumed else ''})",
            "poly": poly, "height": float(height), "height_assumed": bool(assumed),
            "bbox": (min(xs), min(ys), max(xs), max(ys)), "src": src}


def _parse_height(tags):
    h = tags.get("height") or tags.get("building:height")
    if h:
        try:
            return float(str(h).lower().replace("m", "").strip()), False
        except ValueError:
            pass
    lv = tags.get("building:levels")
    if lv:
        try:
            return float(lv) * 3.2, False
        except ValueError:
            pass
    return 10.0, True


def fetch_osm_buildings(frame: GeoFrame, bounds, timeout=10.0, limit=900):
    """Return (buildings, note). Never raises: failures return ([], reason)."""
    x0, y0, x1, y1 = bounds
    s, w = frame.to_geo(x0, y0)
    n, e = frame.to_geo(x1, y1)
    q = f'[out:json][timeout:{int(timeout)}];way["building"]({s:.6f},{w:.6f},{n:.6f},{e:.6f});out geom;'
    try:
        req = urllib.request.Request(
            "https://overpass-api.de/api/interpreter", data=("data=" + urllib.parse.quote(q)).encode(),
            headers={"User-Agent": "ai-drone-path-planner/1.0"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            data = json.loads(r.read().decode("utf-8"))
    except Exception as exc:  # network, timeout, bad JSON, ...
        return [], f"OpenStreetMap building request failed ({type(exc).__name__}: {exc})"
    out = []
    for el in data.get("elements", []):
        geom = el.get("geometry") or []
        if len(geom) < 4:
            continue
        poly = [frame.to_local(g["lat"], g["lon"]) for g in geom]
        if poly[0] == poly[-1]:
            poly = poly[:-1]
        if len(poly) < 3:
            continue
        h, assumed = _parse_height(el.get("tags", {}))
        out.append(_make_building(f"OSM-{el.get('id')}", poly, h, assumed, "osm"))
    if not out:
        return [], "OpenStreetMap returned no buildings for this area"
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    out.sort(key=lambda b: math.hypot((b["bbox"][0] + b["bbox"][2]) / 2 - cx, (b["bbox"][1] + b["bbox"][3]) / 2 - cy))
    return out[:limit], f"{min(len(out), limit)} buildings from OpenStreetMap (heights marked ~ are assumed 10 m)"


def synthetic_buildings(bounds, keep_out, rng, line):
    """Deterministic fallback city: random rotated boxes + two towers near the direct line."""
    x0, y0, x1, y1 = bounds
    area = (x1 - x0) * (y1 - y0)
    n = int(max(25, min(140, area / 25000.0)))
    out, k = [], 0

    def ok(cx, cy, r):
        return all(math.hypot(cx - kx, cy - ky) > kr + r for kx, ky, kr in keep_out)

    def add(cx, cy, w, d, ang, h):
        nonlocal k
        c, s = math.cos(ang), math.sin(ang)
        poly = [(cx + px * c - py * s, cy + px * s + py * c)
                for px, py in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2))]
        k += 1
        out.append(_make_building(f"S{k}", poly, h, False, "synthetic"))

    (sx, sy), (dx, dy) = line
    for f, off in ((0.30, 10.0), (0.62, -12.0)):   # tall towers close to the direct line
        L = math.hypot(dx - sx, dy - sy) or 1.0
        px, py = sx + (dx - sx) * f - (dy - sy) / L * off, sy + (dy - sy) * f + (dx - sx) / L * off
        if ok(px, py, 30):
            add(px, py, 28, 28, rng.uniform(0, 3.14), rng.uniform(85, 100))
    tries = 0
    while len(out) < n and tries < n * 20:
        tries += 1
        cx, cy = rng.uniform(x0, x1), rng.uniform(y0, y1)
        w, d = rng.uniform(18, 46), rng.uniform(18, 46)
        if not ok(cx, cy, max(w, d) * 0.75):
            continue
        add(cx, cy, w, d, rng.uniform(0, 3.14), rng.choice([8, 12, 16, 20, 28, 36, 45, 60, 75, 90]))
    return out


# --------------------------------------------------------------------------
# World (obstacles) and clearance queries
# --------------------------------------------------------------------------
class DynObstacle:
    def __init__(self, oid, name, radius, p0, v):
        self.id, self.name, self.radius, self.p0, self.v = oid, name, radius, p0, v

    def pos(self, t):
        return (self.p0[0] + self.v[0] * t, self.p0[1] + self.v[1] * t, self.p0[2] + self.v[2] * t)


class World:
    def __init__(self, frame, buildings, nofly, margin):
        self.frame, self.buildings, self.nofly, self.margin = frame, buildings, nofly, margin
        self.dynamic: list[DynObstacle] = []
        self._hash: dict = {}
        for bi, b in enumerate(buildings):
            x0, y0, x1, y1 = b["bbox"]
            for ix in range(int(math.floor((x0 - CLEAR_CAP) / HASH)), int(math.floor((x1 + CLEAR_CAP) / HASH)) + 1):
                for iy in range(int(math.floor((y0 - CLEAR_CAP) / HASH)), int(math.floor((y1 + CLEAR_CAP) / HASH)) + 1):
                    self._hash.setdefault((ix, iy), []).append(bi)

    def static_clearance(self, x, y, z):
        """(distance to nearest building prism / no-fly cylinder, that object)."""
        best, who = CLEAR_CAP, None
        for bi in self._hash.get((int(math.floor(x / HASH)), int(math.floor(y / HASH))), ()):
            b = self.buildings[bi]
            x0, y0, x1, y1 = b["bbox"]
            dv = max(0.0, z - b["height"])
            if math.sqrt(max(x0 - x, 0, x - x1) ** 2 + max(y0 - y, 0, y - y1) ** 2 + dv * dv) >= best:
                continue
            d = math.hypot(poly_dist(x, y, b["poly"]), dv)
            if d < best:
                best, who = d, b
        for nz in self.nofly:
            d = max(0.0, math.hypot(x - nz["x"], y - nz["y"]) - nz["radius"])
            if d < best:
                best, who = d, nz
        return best, who

    def clearance(self, x, y, z, t=None):
        """(distance, object name or None). Includes moving obstacles when t is given."""
        best, who = self.static_clearance(x, y, z)
        name = who["name"] if who else None
        if t is not None:
            for ob in self.dynamic:
                d = max(0.0, dist3((x, y, z), ob.pos(t)) - ob.radius)
                if d < best:
                    best, name = d, ob.name
        return best, name


def risk_level(clearance, margin):
    return "critical" if clearance < margin else ("caution" if clearance < 3 * margin else "clear")


# --------------------------------------------------------------------------
# 3D occupancy grid + A*
# --------------------------------------------------------------------------
class Grid:
    """Layers sit at z = k*dz (k = 1..kmax) so the cruise layer is EXACTLY the requested altitude."""

    def __init__(self, bounds, cs, cruise_alt, ceiling):
        self.xmin, self.ymin, xmax, ymax = bounds
        self.cs = cs
        self.nx, self.ny = int(math.ceil((xmax - self.xmin) / cs)), int(math.ceil((ymax - self.ymin) / cs))
        self.kc = max(1, round(cruise_alt / 10.0))
        self.dz = cruise_alt / self.kc
        self.kmax = max(self.kc, int(math.floor(ceiling / self.dz + 1e-9)))
        self.near: dict = {}
        self.blocked: set = set()
        self.R = 0.0

    def cell(self, x, y, z):
        i = min(self.nx - 1, max(0, int((x - self.xmin) // self.cs)))
        j = min(self.ny - 1, max(0, int((y - self.ymin) // self.cs)))
        k = min(self.kmax, max(1, int(round(z / self.dz))))
        return (i, j, k)

    def center(self, c):
        return (self.xmin + (c[0] + 0.5) * self.cs, self.ymin + (c[1] + 0.5) * self.cs, c[2] * self.dz)

    def build(self, world, margin):
        cs, dz, R = self.cs, self.dz, 3 * margin + self.cs
        self.R = R
        hb = margin + 0.5 * cs
        vb = margin + 0.5 * dz

        def mark(key, eff, hard):
            if eff < self.near.get(key, 1e18):
                self.near[key] = eff
            if hard:
                self.blocked.add(key)

        def col_range(x0, y0, x1, y1):
            i0, j0, _ = self.cell(x0 - R, y0 - R, self.dz)
            i1, j1, _ = self.cell(x1 + R, y1 + R, self.dz)
            return range(i0, i1 + 1), range(j0, j1 + 1)

        for b in world.buildings:
            x0, y0, x1, y1 = b["bbox"]
            ri, rj = col_range(x0, y0, x1, y1)
            for i in ri:
                for j in rj:
                    cx, cy, _ = self.center((i, j, 1))
                    if math.hypot(max(x0 - cx, 0, cx - x1), max(y0 - cy, 0, cy - y1)) > R:
                        continue
                    dh = poly_dist(cx, cy, b["poly"])
                    for k in range(1, self.kmax + 1):
                        gap = k * dz - b["height"]
                        eff = math.hypot(dh, max(0.0, gap))
                        if eff <= R:
                            mark((i, j, k), eff, dh <= hb and gap < vb)
        for nz in world.nofly:
            r = nz["radius"]
            ri, rj = col_range(nz["x"] - r, nz["y"] - r, nz["x"] + r, nz["y"] + r)
            for i in ri:
                for j in rj:
                    cx, cy, _ = self.center((i, j, 1))
                    d = max(0.0, math.hypot(cx - nz["x"], cy - nz["y"]) - r)
                    if d <= R:
                        for k in range(1, self.kmax + 1):
                            mark((i, j, k), d, d <= hb)


NEIGHBOURS = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0)]


def astar(grid: Grid, start, goal, cfg: Config, extra=frozenset(), free=frozenset(), max_nodes=400000):
    """3D A* over the occupancy grid. Returns (cells | None, nodes_expanded)."""
    cs, dz = grid.cs, grid.dz
    gx, gy, gz = grid.center(goal)
    blocked, near, R = grid.blocked, grid.near, grid.R
    wd, wc, wr = cfg.w_dist, cfg.w_climb, cfg.w_risk

    def h(c):
        x, y, z = grid.center(c)
        return wd * math.sqrt((x - gx) ** 2 + (y - gy) ** 2 + (z - gz) ** 2)

    cnt = itertools.count()
    openq = [(h(start), 0.0, next(cnt), start)]
    best_g, parent, expanded = {start: 0.0}, {}, 0
    while openq:
        _, g, _, cur = heapq.heappop(openq)
        if g > best_g.get(cur, 1e18) + 1e-9:
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            return path[::-1], expanded
        expanded += 1
        if expanded > max_nodes:
            return None, expanded
        for di, dj, dk in NEIGHBOURS:
            nb = (cur[0] + di, cur[1] + dj, cur[2] + dk)
            if not (0 <= nb[0] < grid.nx and 0 <= nb[1] < grid.ny and 1 <= nb[2] <= grid.kmax):
                continue
            if nb != goal and nb not in free and (nb in blocked or nb in extra):
                continue
            step = math.sqrt((di * cs) ** 2 + (dj * cs) ** 2 + (dk * dz) ** 2)
            pen = max(0.0, 1.0 - near.get(nb, 1e18) / R)
            ng = g + wd * step + wc * abs(dk) * dz + wr * pen * step
            if ng < best_g.get(nb, 1e18) - 1e-9:
                best_g[nb] = ng
                parent[nb] = cur
                heapq.heappush(openq, (ng + h(nb), ng, next(cnt), nb))
    return None, expanded


# --------------------------------------------------------------------------
# Energy / flight time
# --------------------------------------------------------------------------
def hover_power_w(cfg: Config):
    W = (cfg.drone_mass_kg + cfg.payload_kg) * G
    return W ** 1.5 / math.sqrt(2 * RHO * ROTOR_AREA) / ETA_PROP + P_AVIONICS


def power_w(cfg: Config, vgx, vgy, vz):
    """Electrical power (W) for a ground velocity (vgx, vgy) and climb rate vz in a wind."""
    W = (cfg.drone_mass_kg + cfg.payload_kg) * G
    p_h = W ** 1.5 / math.sqrt(2 * RHO * ROTOR_AREA) / ETA_PROP
    v_air = math.hypot(vgx - cfg.wind_east_ms, vgy - cfg.wind_north_ms)
    p = p_h + 0.5 * RHO * CD_A * v_air ** 3 / ETA_PROP + W * vz / ETA_PROP
    return max(p, 0.3 * p_h) + P_AVIONICS


# --------------------------------------------------------------------------
# Routes (waypoints + legs)
# --------------------------------------------------------------------------
class Route:
    """A list of 3D waypoints (local metres) with per-leg geometry, speed, energy and explanations."""

    def __init__(self, wps, world, cfg, dest_xy, ver, t_start, cum_offset, reason, first_is_takeoff):
        self.wps, self.ver, self.t_start, self.reason = [tuple(w) for w in wps], ver, t_start, reason
        self.cum_offset = cum_offset
        fr = world.frame
        self.legs, cum = [], 0.0
        n = len(self.wps) - 1
        bdir = None
        for i in range(n):
            a, b = self.wps[i], self.wps[i + 1]
            dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            horiz, length = math.hypot(dx, dy), math.sqrt(dx * dx + dy * dy + dz * dz)
            if length < 1e-9:
                continue
            slope = dz / length
            if i == 0 and first_is_takeoff:
                phase = "takeoff"
            elif i == n - 1 and b[2] < 1e-6 and dz < 0:
                phase = "landing"
            elif slope > 0.1:
                phase = "climbing"
            elif slope < -0.1:
                phase = "descending"
            else:
                phase = "cruising"
            speed = cfg.speed_ms if abs(dz) < 1e-9 else min(cfg.speed_ms, cfg.vz_max_ms * length / abs(dz))
            vgx, vgy, vz = dx / length * speed, dy / length * speed, dz / length * speed
            pw = power_w(cfg, vgx, vgy, vz)
            # sample the leg for its closest static obstacle
            ns = max(1, int(math.ceil(length / 4.0)))
            mc, mo = CLEAR_CAP, None
            for s in range(ns + 1):
                u = s / ns
                c, o = world.static_clearance(a[0] + dx * u, a[1] + dy * u, a[2] + dz * u)
                if c < mc:
                    mc, mo = c, (o["name"] if o else None)
            lat0, lon0 = fr.to_geo(a[0], a[1])
            lat1, lon1 = fr.to_geo(b[0], b[1])
            hd = bearing_deg(dx, dy) if horiz > 0.5 else None
            direct = bearing_deg(dest_xy[0] - a[0], dest_xy[1] - a[1])
            leg = {
                "idx": len(self.legs), "a": i, "b": i + 1, "phase": phase,
                "x0": a[0], "y0": a[1], "z0": a[2], "x1": b[0], "y1": b[1], "z1": b[2],
                "lat0": lat0, "lon0": lon0, "lat1": lat1, "lon1": lon1,
                "horiz_m": horiz, "vert_m": dz, "length_m": length,
                "cum_start_m": cum, "cum_end_m": cum + length,
                "cum_start_takeoff_m": cum_offset + cum, "cum_end_takeoff_m": cum_offset + cum + length,
                "speed_ms": speed, "time_s": length / speed, "power_w": pw,
                "energy_wh": pw * (length / speed) / 3600.0,
                "heading_deg": hd, "min_clearance_m": mc, "clearance_obj": mo,
            }
            leg["why"] = self._why(leg, direct, cfg)
            cum += length
            self.legs.append(leg)
        self.total_m = cum
        self.horizontal_m = sum(l["horiz_m"] for l in self.legs)
        self.time_s = sum(l["time_s"] for l in self.legs)
        self.energy_wh = sum(l["energy_wh"] for l in self.legs)
        self._waypoints(world, cfg)

    @staticmethod
    def _why(l, direct, cfg):
        near = (f" Closest obstacle: {l['clearance_obj']} at {l['min_clearance_m']:.1f} m."
                if l["clearance_obj"] else f" No obstacle within {CLEAR_CAP:.0f} m.")
        ph = l["phase"]
        if ph == "takeoff":
            return (f"Vertical takeoff: climb {l['vert_m']:.0f} m straight up to {l['z1']:.0f} m AGL "
                    f"so the drone is above rooftops before it moves sideways." + near)
        if ph == "landing":
            return f"Final descent: straight down {-l['vert_m']:.0f} m over the destination to the ground." + near
        if ph == "climbing":
            return (f"Climbing {l['vert_m']:.0f} m (to {l['z1']:.0f} m AGL) over {l['horiz_m']:.0f} m of ground: "
                    f"the planner buys altitude where going over is cheaper than going around." + near)
        if ph == "descending":
            return (f"Descending {-l['vert_m']:.0f} m (to {l['z1']:.0f} m AGL) over {l['horiz_m']:.0f} m of ground: "
                    f"no need to stay high once the obstacle is behind." + near)
        hd = l["heading_deg"]
        if hd is not None and l["horiz_m"] > 5:
            off = abs((hd - direct + 180) % 360 - 180)
            if off > 20:
                return (f"Detour: heading {hd:.0f}° while the direct bearing to the destination is {direct:.0f}° "
                        f"({off:.0f}° off) to keep at least {cfg.safety_margin_m:.0f} m from obstacles." + near)
        return f"Cruising {l['horiz_m']:.0f} m at {l['z1']:.0f} m AGL toward the destination." + near

    def _waypoints(self, world, cfg):
        fr, self.waypoints = world.frame, []
        n = len(self.wps)
        cum = 0.0
        for i, w in enumerate(self.wps):
            if i > 0:
                cum += self.legs[i - 1]["length_m"]
            lat, lon = fr.to_geo(w[0], w[1])
            c, o = world.static_clearance(*w)
            if self.ver == 0 and i == 0:
                label, kind, why = "W0 Start", "start", "Takeoff point (ground level, z = 0 m)."
            elif i == n - 1:
                label, kind, why = f"W{i} Destination", "destination", "Delivery point: the drone touches down here."
            else:
                label = f"W{i}" if self.ver == 0 else f"R{self.ver}-{i}"
                prev_l, next_l = self.legs[i - 1], self.legs[i]
                kind, why = "turn", ""
                if self.ver > 0 and i == 0:
                    kind, why = "reroute_start", "Re-route starts here: the drone leaves the old route at this point."
                elif prev_l["phase"] == "takeoff":
                    kind, why = "top_of_climb", f"End of the vertical climb: {w[2]:.0f} m AGL cruise layer reached."
                elif next_l["phase"] == "landing":
                    kind, why = "top_of_descent", "Above the destination: final descent begins."
                else:
                    h0, h1 = prev_l["heading_deg"], next_l["heading_deg"]
                    if h0 is not None and h1 is not None and abs((h1 - h0 + 180) % 360 - 180) > 10:
                        kind = "turn"
                        why = f"Heading change of {abs((h1 - h0 + 180) % 360 - 180):.0f}° to steer around an obstacle."
                    else:
                        kind = "altitude_change"
                        why = f"Altitude change ({prev_l['z1']:.0f} → {next_l['z1']:.0f} m AGL) to pass an obstacle."
                    if o:
                        why += f" Nearest: {o['name']} at {c:.1f} m."
            self.waypoints.append({
                "idx": i, "label": label, "kind": kind, "why": why, "x": w[0], "y": w[1], "z": w[2],
                "lat": lat, "lon": lon, "cum_m": cum, "cum_takeoff_m": self.cum_offset + cum,
                "clearance_m": c, "clearance_obj": o["name"] if o else None,
            })

    def to_dict(self):
        return {"ver": self.ver, "t_start": self.t_start, "reason": self.reason, "total_m": self.total_m,
                "horizontal_m": self.horizontal_m, "time_s": self.time_s, "energy_wh": self.energy_wh,
                "cum_offset_m": self.cum_offset, "waypoints": self.waypoints, "legs": self.legs}

    def pos_after(self, leg_i, off, tau):
        """Position after flying for tau seconds starting `off` metres into leg `leg_i`."""
        li, o = leg_i, off
        while li < len(self.legs):
            l = self.legs[li]
            left_t = (l["length_m"] - o) / l["speed_ms"]
            if tau < left_t:
                u = (o + tau * l["speed_ms"]) / l["length_m"]
                return (l["x0"] + (l["x1"] - l["x0"]) * u, l["y0"] + (l["y1"] - l["y0"]) * u,
                        l["z0"] + (l["z1"] - l["z0"]) * u)
            tau -= left_t
            li += 1
            o = 0.0
        return self.wps[-1]


# --------------------------------------------------------------------------
# Planning helpers (initial plan and re-plans)
# --------------------------------------------------------------------------
def _segment_ok(world, grid, cfg, a, b, extra):
    step = max(1.0, min(grid.cs / 3.0, 5.0))
    n = max(1, int(math.ceil(dist3(a, b) / step)))
    zmin = grid.dz * 0.999
    for s in range(n + 1):
        u = s / n
        p = (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u)
        if p[2] < min(a[2], b[2], zmin) - 1e-6 or p[2] > grid.kmax * grid.dz + 1e-6:
            return False
        if world.static_clearance(*p)[0] < cfg.safety_margin_m - 1e-6:
            return False
        if extra and grid.cell(*p) in extra:
            return False
    return True


def _simplify(world, grid, cfg, pts, extra):
    # 1) drop collinear points
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        a, b, c = out[-1], pts[i], pts[i + 1]
        u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        v = (c[0] - b[0], c[1] - b[1], c[2] - b[2])
        cr = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        if math.sqrt(cr[0] ** 2 + cr[1] ** 2 + cr[2] ** 2) > 1e-6 * max(1.0, dist3(a, b) * dist3(b, c)):
            out.append(b)
    out.append(pts[-1])
    # 2) greedy line-of-sight shortcuts (continuous clearance, not just grid cells)
    res, i = [out[0]], 0
    while i < len(out) - 1:
        j = len(out) - 1
        while j > i + 1 and not _segment_ok(world, grid, cfg, out[i], out[j], extra):
            j -= 1
        res.append(out[j])
        i = j
    return res


def plan_flight_nodes(world, grid, cfg, begin, goal_xy, extra=frozenset(), free=frozenset()):
    """A* from `begin` (x,y,z) to above the destination at cruise altitude. Returns (nodes, expanded)."""
    zc = grid.kc * grid.dz
    sc = grid.cell(*begin)
    gc = grid.cell(goal_xy[0], goal_xy[1], zc)
    cells, nodes = astar(grid, sc, gc, cfg, extra, free | {sc})
    if cells is None:
        return None, nodes
    pts = [tuple(begin)] + [grid.center(c) for c in cells[1:-1]] + [(goal_xy[0], goal_xy[1], zc)]
    return _simplify(world, grid, cfg, pts, extra), nodes


# --------------------------------------------------------------------------
# Dynamic obstacles / threat detection
# --------------------------------------------------------------------------
def make_dynamic(route: Route, cfg: Config):
    obs = []
    for n, (frac, side, radius) in enumerate(((0.42, 1.0, 10.0), (0.72, -1.0, 8.0)), 1):
        tc = max(8.0, frac * route.time_s)
        P = route.pos_after(0, 0.0, tc)
        li, acc = 0, 0.0
        while li < len(route.legs) - 1 and acc + route.legs[li]["time_s"] < tc:
            acc += route.legs[li]["time_s"]
            li += 1
        l = route.legs[li]
        hx, hy = l["x1"] - l["x0"], l["y1"] - l["y0"]
        if math.hypot(hx, hy) < 1.0:
            hx, hy = route.wps[-1][0] - route.wps[0][0], route.wps[-1][1] - route.wps[0][1]
        hl = math.hypot(hx, hy) or 1.0
        px, py = -hy / hl * side, hx / hl * side
        speed = max(3.0, min(9.0, 220.0 / tc))
        L0 = speed * tc
        z = max(20.0, P[2])
        obs.append(DynObstacle(f"D{n}", f"Moving obstacle {n}", radius, (P[0] - px * L0, P[1] - py * L0, z),
                               (px * speed, py * speed, 0.0)))
    return obs


def _horizon(cfg):
    return min(25.0, max(6.0, cfg.detect_range_m / max(cfg.speed_ms, 1.0) + 6.0))


def detect_threat(world, cfg, route, leg_i, off, pos, t):
    thr_m = 2.0 * cfg.safety_margin_m
    H, best = _horizon(cfg), None
    for ob in world.dynamic:
        if dist3(pos, ob.pos(t)) - ob.radius > cfg.detect_range_m:
            continue
        for k in range(int(H / 0.5) + 1):
            tau = k * 0.5
            p = route.pos_after(leg_i, off, tau)
            d = dist3(p, ob.pos(t + tau)) - ob.radius
            if d < thr_m:
                if best is None or tau < best["tau"]:
                    best = {"ob": ob, "tau": tau, "clear": d, "point": p}
                break
    return best


def _sphere_cells(grid, ob, t, span, thr_m):
    cells = set()
    rad = ob.radius + thr_m + 0.5 * max(grid.cs, grid.dz)
    for tau in range(0, int(span) + 1):
        q = ob.pos(t + tau)
        i0, j0, k0 = grid.cell(q[0] - rad, q[1] - rad, q[2] - rad)
        i1, j1, k1 = grid.cell(q[0] + rad, q[1] + rad, q[2] + rad)
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                for k in range(k0, k1 + 1):
                    c = grid.center((i, j, k))
                    if dist3(c, q) <= rad:
                        cells.add((i, j, k))
    return cells


# --------------------------------------------------------------------------
# Flight simulation
# --------------------------------------------------------------------------
def fly(world, grid, cfg, route0, free, dest_xy):
    fr = world.frame
    margin = cfg.safety_margin_m
    thr_m = 2.0 * margin
    routes = [route0]
    route, leg_i, off = route0, 0, 0.0
    pos = tuple(route0.wps[0])
    t = dist = e_used = hold_time = 0.0
    frames, events = [], []
    plan_nodes = 0
    dt = min(3.0, max(1.0, route0.time_s / 400.0))
    t_max = route0.time_s * 3 + cfg.max_hold_s * 12 + 300
    hold = None
    replans = 0
    failure = None
    hover_w = hover_power_w(cfg)

    def remaining():
        if leg_i >= len(route.legs):
            return 0.0
        return route.legs[leg_i]["length_m"] - off + sum(l["length_m"] for l in route.legs[leg_i + 1:])

    def leg_text(prefix=""):
        l = route.legs[leg_i]
        return f"{prefix}{PHASE_NAME[l['phase']]} · leg {leg_i + 1}/{len(route.legs)}: {l['why']}"

    def add(state, text, ev=None, speed=0.0):
        lat, lon = fr.to_geo(pos[0], pos[1])
        c, o = world.clearance(pos[0], pos[1], pos[2], t)
        frames.append({
            "t": t, "x": pos[0], "y": pos[1], "z": pos[2], "lat": lat, "lon": lon,
            "dist": dist, "rem": remaining(), "e_used": e_used,
            "e_left": max(0.0, cfg.battery_wh - e_used),
            "batt": max(0.0, 100.0 * (1 - e_used / cfg.battery_wh)),
            "speed": speed, "state": state, "text": text, "clr": c, "clr_obj": o,
            "risk": risk_level(c, margin), "ver": route.ver, "leg": min(leg_i, len(route.legs) - 1), "ev": ev,
        })

    def event(kind, reason, detail, obstacle=None, **extra):
        lat, lon = fr.to_geo(pos[0], pos[1])
        ev = {"id": f"E{len(events) + 1}", "type": kind, "t": t, "x": pos[0], "y": pos[1], "z": pos[2],
              "lat": lat, "lon": lon, "alt_m": pos[2], "cum_m": dist, "reason": reason, "detail": detail,
              "obstacle": obstacle}
        ev.update(extra)
        events.append(ev)
        return ev

    def try_reroute(thr):
        nonlocal plan_nodes
        span = _horizon(cfg) + 4
        extra = set()
        for ob in world.dynamic:
            if dist3(pos, ob.pos(t)) - ob.radius <= cfg.detect_range_m * 1.5:
                extra |= _sphere_cells(grid, ob, t, span, thr_m)
        nodes, n_exp = plan_flight_nodes(world, grid, cfg, pos, dest_xy, frozenset(extra), free)
        plan_nodes += n_exp
        if nodes is None:
            return None
        new = Route(nodes + [(dest_xy[0], dest_xy[1], 0.0)], world, cfg, dest_xy, len(routes), t, dist,
                    "", False)
        return new

    def start_reroute(new, thr, reason_prefix):
        nonlocal route, leg_i, off, replans
        old_rem = remaining()
        delta = new.total_m - old_rem
        new.reason = (f"{reason_prefix}{thr['ob'].name} was predicted to pass within {thr['clear']:.1f} m of the drone "
                      f"in {thr['tau']:.1f} s (required ≥ {thr_m:.0f} m).")
        ev = event("reroute", new.reason,
                   f"Remaining distance changes from {old_rem:.1f} m to {new.total_m:.1f} m ({delta:+.1f} m).",
                   thr["ob"].name, remaining_before_m=old_rem, remaining_after_m=new.total_m, delta_m=delta,
                   route_ver=new.ver)
        routes.append(new)
        route, leg_i, off = new, 0, 0.0
        replans += 1
        add("rerouting", f"Re-routing around {thr['ob'].name}: {new.reason} {ev['detail']}", ev["id"])

    add(route.legs[0]["phase"], leg_text("Start · "))
    guard = 0
    while leg_i < len(route.legs):
        guard += 1
        if guard > 30000 or t > t_max:
            failure = "Simulation time limit reached before the drone arrived."
            break
        if e_used >= cfg.battery_wh:
            failure = "Battery depleted before reaching the destination."
            break

        thr = detect_threat(world, cfg, route, leg_i, off, pos, t) if world.dynamic else None
        landing = route.legs[leg_i]["phase"] == "landing"

        if hold is not None:
            if thr is None:
                ev = event("resume", f"Path is clear again after holding {t - hold['t0']:.1f} s.",
                           f"Flight resumes with {remaining():.1f} m to go.", hold["name"], hold_s=t - hold["t0"])
                hold_time += t - hold["t0"]
                hold = None
                add(route.legs[leg_i]["phase"], leg_text("Resuming · "), ev["id"])
            elif t - hold["t0"] >= cfg.max_hold_s:
                new = None if landing else try_reroute(thr)
                if new is None:
                    failure = f"{thr['ob'].name} did not clear the path within {cfg.max_hold_s:.0f} s and no re-route exists."
                    break
                hold_time += t - hold["t0"]
                hold = None
                start_reroute(new, thr, "Hold timed out. ")
                continue
            else:
                e_used += hover_w * dt / 3600.0
                t += dt
                add("holding", f"Holding position: {thr['ob'].name} would come within {thr['clear']:.1f} m of the route "
                    f"in {thr['tau']:.1f} s. Waiting for it to clear ({t - hold['t0']:.0f}/{cfg.max_hold_s:.0f} s).")
                continue
        elif thr is not None:
            new = None if (landing or replans >= 6 or cfg.response == "hold") else try_reroute(thr)
            rem0 = remaining()
            attractive = new is not None and (new.total_m - rem0) <= max(0.25 * rem0, 40.0)
            safe_hold = all(dist3(pos, thr["ob"].pos(t + k)) - thr["ob"].radius > thr_m
                            for k in range(0, int(cfg.max_hold_s) + 1))
            if cfg.response == "reroute":
                choose = "reroute" if new else ("hold" if safe_hold else None)
            elif cfg.response == "hold":
                choose = "hold" if safe_hold else ("reroute" if new or not landing else None)
                if choose == "reroute" and new is None:
                    new = None if landing else try_reroute(thr)
                    choose = "reroute" if new else None
            else:
                choose = "reroute" if attractive else ("hold" if safe_hold else ("reroute" if new else None))
            if choose == "reroute" and new is not None:
                start_reroute(new, thr, "")
                continue
            if choose == "hold":
                why = ("re-route not possible" if new is None else
                       f"a re-route would add {new.total_m - rem0:.1f} m, so waiting is cheaper") \
                    if cfg.response != "hold" else "hold response selected"
                ev = event("hold", f"{thr['ob'].name} is predicted to pass within {thr['clear']:.1f} m of the route "
                           f"in {thr['tau']:.1f} s; holding because {why}.",
                           f"Distance impact: none (0.0 m added); costs flight time and hover energy. "
                           f"Remaining distance stays {rem0:.1f} m.", thr["ob"].name)
                hold = {"t0": t, "name": thr["ob"].name}
                e_used += hover_w * dt / 3600.0
                t += dt
                add("holding", ev["reason"], ev["id"])
                continue
            failure = f"{thr['ob'].name} blocks the route and neither a re-route nor a safe hold is possible."
            break

        # ---- normal motion for one step
        left_dt = dt
        while left_dt > 1e-9 and leg_i < len(route.legs):
            l = route.legs[leg_i]
            need = (l["length_m"] - off) / l["speed_ms"]
            if need <= left_dt + 1e-9:
                d = l["length_m"] - off
                e_used += l["power_w"] * need / 3600.0
                dist += d
                t += need
                left_dt -= need
                pos = (l["x1"], l["y1"], l["z1"])
                off, leg_i = 0.0, leg_i + 1
                if leg_i >= len(route.legs):
                    add("landed", f"Landed at the destination after {dist:.1f} m flown in {t:.0f} s.")
                else:
                    wp = route.waypoints[l["b"]]
                    add(route.legs[leg_i]["phase"], leg_text(f"Reached {wp['label']} ({wp['cum_takeoff_m']:.1f} m from takeoff). Next → "),
                        speed=route.legs[leg_i]["speed_ms"])
                    frames[-1]["speed"] = route.legs[leg_i]["speed_ms"]
            else:
                d = l["speed_ms"] * left_dt
                e_used += l["power_w"] * left_dt / 3600.0
                off += d
                dist += d
                t += left_dt
                left_dt = 0.0
                u = off / l["length_m"]
                pos = (l["x0"] + (l["x1"] - l["x0"]) * u, l["y0"] + (l["y1"] - l["y0"]) * u,
                       l["z0"] + (l["z1"] - l["z0"]) * u)
                add(l["phase"], leg_text(), speed=l["speed_ms"])
    if failure:
        event("abort", failure, "The simulation stopped here.")
        add("aborted", f"Aborted: {failure}", events[-1]["id"])
    # frame "speed" describes the motion that ENDS at the frame; keep the start frame readable
    return {"frames": frames, "events": events, "routes": routes, "dist": dist, "t": t, "e_used": e_used,
            "hold_time": hold_time + (t - hold["t0"] if hold else 0.0), "failure": failure,
            "plan_nodes": plan_nodes}


# --------------------------------------------------------------------------
# Public API
# --------------------------------------------------------------------------
def validate_config(cfg: Config):
    errs = []
    (la, lo), (lb, lb2) = cfg.start, cfg.dest
    for name, v in (("start", cfg.start), ("destination", cfg.dest)):
        if not (-85 <= v[0] <= 85 and -180 <= v[1] <= 180):
            errs.append(f"The {name} coordinates {v} are not valid latitude/longitude.")
    if errs:
        return errs
    d = haversine_m(la, lo, lb, lb2)
    if d < 40:
        errs.append(f"Start and destination are only {d:.0f} m apart - pick points at least 40 m apart.")
    if d > 6000:
        errs.append(f"Start and destination are {d / 1000:.1f} km apart; this planner supports routes up to 6 km.")
    if not 10 <= cfg.altitude_m <= 150:
        errs.append("Cruise altitude must be between 10 and 150 m AGL.")
    if cfg.ceiling_m < cfg.altitude_m:
        errs.append("The planning ceiling must not be lower than the cruise altitude.")
    if cfg.speed_ms < 2 or cfg.speed_ms > 30:
        errs.append("Speed must be between 2 and 30 m/s.")
    if cfg.drone_mass_kg <= 0 or cfg.payload_kg < 0 or cfg.battery_wh <= 0:
        errs.append("Drone mass and battery capacity must be positive; payload cannot be negative.")
    if cfg.w_dist <= 0:
        errs.append("The distance cost weight must be greater than 0.")
    if cfg.safety_margin_m < 1:
        errs.append("Safety margin must be at least 1 m.")
    return errs


def run_simulation(cfg: Config) -> dict:
    """Plan and fly the mission. Never raises: problems come back as result['ok'] = False + result['error']."""
    try:
        return _run(cfg)
    except SimError as exc:
        return {"ok": False, "error": str(exc)}
    except Exception as exc:  # unexpected bug: still show something readable
        return {"ok": False, "error": f"Unexpected simulation error ({type(exc).__name__}: {exc})."}


def _run(cfg: Config) -> dict:
    errs = validate_config(cfg)
    if errs:
        raise SimError(" ".join(errs))
    t_wall = time.time()
    frame = GeoFrame(*cfg.start)
    sx, sy = 0.0, 0.0
    dx, dy = frame.to_local(*cfg.dest)
    D = math.hypot(dx, dy)
    pad = max(150.0, 0.35 * D)
    bounds = (min(sx, dx) - pad, min(sy, dy) - pad, max(sx, dx) + pad, max(sy, dy) + pad)
    cs = max(10.0, round(max(bounds[2] - bounds[0], bounds[3] - bounds[1]) / 110.0, 1))
    seed = cfg.seed if cfg.seed is not None else int(abs(cfg.start[0]) * 1e4 * 7 + abs(cfg.start[1]) * 1e4 * 13
                                                       + abs(cfg.dest[0]) * 1e4 * 3)
    rng = random.Random(seed)
    warnings = []

    buildings, note, source = [], "", "synthetic"
    if cfg.use_osm:
        buildings, note = fetch_osm_buildings(frame, bounds, cfg.overpass_timeout_s)
        if buildings:
            source = "OpenStreetMap"
    if not buildings:
        buildings = synthetic_buildings(bounds, [(sx, sy, 45.0), (dx, dy, 45.0)], rng, ((sx, sy), (dx, dy)))
        why = note or "OpenStreetMap lookup was turned off"
        note = f"{why}. Using {len(buildings)} synthetic buildings instead (demo data, not the real city)."
        warnings.append(note)

    nofly = []
    if cfg.use_nofly:
        ux, uy = dx / D, dy / D
        for n, (f, side) in enumerate(((0.35, 1.0), (0.70, -1.0)), 1):
            r = max(40.0, min(90.0, 0.05 * D))
            off = side * 0.06 * D
            cx, cy = sx + dx * f - uy * off, sy + dy * f + ux * off
            if math.hypot(cx - sx, cy - sy) > r + 50 and math.hypot(cx - dx, cy - dy) > r + 50:
                lat, lon = frame.to_geo(cx, cy)
                nofly.append({"id": f"NF{n}", "name": f"No-fly zone {'AB'[n - 1]}", "x": cx, "y": cy, "lat": lat,
                              "lon": lon, "radius": r, "top": cfg.ceiling_m})

    world = World(frame, buildings, nofly, cfg.safety_margin_m)

    # takeoff / landing columns must be usable
    for label, (px, py) in (("start", (sx, sy)), ("destination", (dx, dy))):
        worst, who = CLEAR_CAP, None
        for z in [i * 2.0 for i in range(int(cfg.altitude_m // 2) + 1)]:
            c, o = world.static_clearance(px, py, z)
            if c < worst:
                worst, who = c, o
        if worst < 1.0:
            raise SimError(f"The {label} point is inside or touching {who['name']}. Click open ground or a street "
                           f"instead of a building / no-fly zone.")
        if worst < cfg.safety_margin_m:
            warnings.append(f"The {label} point is only {worst:.1f} m from {who['name']} "
                            f"(safety margin {cfg.safety_margin_m:.0f} m); the vertical climb/descent is tight.")

    grid = Grid(bounds, cs, cfg.altitude_m, cfg.ceiling_m)
    grid.build(world, cfg.safety_margin_m)
    free = frozenset((c[0], c[1], k) for c in (grid.cell(sx, sy, grid.dz), grid.cell(dx, dy, grid.dz))
                     for k in range(1, grid.kmax + 1))

    nodes, n_exp = plan_flight_nodes(world, grid, cfg, (sx, sy, grid.kc * grid.dz), (dx, dy), frozenset(), free)
    if nodes is None:
        raise SimError("No collision-free route was found between these points "
                       f"(searched {n_exp:,} grid cells). Try a higher cruise altitude / planning ceiling, a smaller "
                       "safety margin, turn off no-fly zones, or choose different points.")
    plan_ms = (time.time() - t_wall) * 1000.0
    route0 = Route([(sx, sy, 0.0)] + nodes + [(dx, dy, 0.0)], world, cfg, (dx, dy), 0, 0.0, 0.0,
                   "Initial plan (A* over the 3D occupancy grid, then line-of-sight smoothing).", True)

    if cfg.use_dynamic:
        world.dynamic = make_dynamic(route0, cfg)

    out = fly(world, grid, cfg, route0, free, (dx, dy))
    frames, events, routes = out["frames"], out["events"], out["routes"]
    if route0.energy_wh > cfg.battery_wh * (1 - cfg.reserve_pct / 100.0):
        warnings.append(f"Planned energy {route0.energy_wh:.1f} Wh exceeds the usable battery "
                        f"({cfg.battery_wh * (1 - cfg.reserve_pct / 100.0):.1f} Wh after the {cfg.reserve_pct:.0f}% reserve).")
    if math.hypot(cfg.wind_east_ms, cfg.wind_north_ms) > 0.8 * cfg.speed_ms:
        warnings.append("Wind is strong compared with the drone's speed; energy use will be high.")

    lat_s, lon_s = cfg.start
    lat_d, lon_d = cfg.dest
    min_clr = min(f["clr"] for f in frames)
    arrived = out["failure"] is None
    metrics = {
        "success": bool(arrived and min_clr > 0.0),
        "planned_distance_m": route0.total_m,
        "straight_line_m": D,
        "actual_distance_m": out["dist"],
        "detour_factor": route0.total_m / D,
        "actual_minus_planned_m": out["dist"] - route0.total_m,
        "planned_time_s": route0.time_s,
        "flight_time_s": out["t"],
        "hold_time_s": out["hold_time"],
        "planned_energy_wh": route0.energy_wh,
        "energy_used_wh": out["e_used"],
        "battery_wh": cfg.battery_wh,
        "battery_left_wh": max(0.0, cfg.battery_wh - out["e_used"]),
        "battery_left_pct": max(0.0, 100.0 * (1 - out["e_used"] / cfg.battery_wh)),
        "max_altitude_m": max(f["z"] for f in frames),
        "min_clearance_m": min_clr,
        "reroute_count": sum(1 for e in events if e["type"] == "reroute"),
        "hold_count": sum(1 for e in events if e["type"] == "hold"),
        "avg_ground_speed_ms": out["dist"] / out["t"] if out["t"] > 0 else 0.0,
        "nodes_expanded": n_exp + out["plan_nodes"],
        "planning_ms": plan_ms,
        "building_count": len(buildings),
        "nofly_count": len(nofly),
        "dynamic_count": len(world.dynamic),
        "haversine_straight_m": haversine_m(lat_s, lon_s, lat_d, lon_d),
    }

    actual = [[f["lon"], f["lat"], f["z"], f["dist"]] for f in frames]
    T = out["t"]
    traces = []
    for ob in world.dynamic:
        pts, k = [], 0
        step = max(1.0, T / 80.0)
        while k * step <= T + step:
            p = ob.pos(k * step)
            la, lo = frame.to_geo(p[0], p[1])
            pts.append([lo, la, p[2]])
            k += 1
        traces.append({"id": ob.id, "name": ob.name, "path": pts})

    bl = []
    for b in buildings:
        bl.append({"id": b["id"], "name": b["name"], "height": b["height"], "assumed": b["height_assumed"],
                   "src": b["src"],
                   "polygon": [[frame.to_geo(px, py)[1], frame.to_geo(px, py)[0]] for px, py in b["poly"]]})
    nf = []
    for z in nofly:
        ring = []
        for k in range(48):
            a = 2 * math.pi * k / 48
            la, lo = frame.to_geo(z["x"] + z["radius"] * math.cos(a), z["y"] + z["radius"] * math.sin(a))
            ring.append([lo, la])
        nf.append(dict(z, polygon=ring))
    dyn = []
    for ob in world.dynamic:
        dyn.append({"id": ob.id, "name": ob.name, "radius": ob.radius, "p0": list(ob.p0), "v": list(ob.v)})

    mid = frame.to_geo((bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2)
    extent = max(bounds[2] - bounds[0], bounds[3] - bounds[1])
    zoom = max(10.0, min(19.0, math.log2(156543.03 * math.cos(math.radians(mid[0])) * 700.0 / (extent * 1.1))))
    sw, ne = frame.to_geo(bounds[0], bounds[1]), frame.to_geo(bounds[2], bounds[3])
    result = {
        "ok": True, "error": None, "failure": out["failure"], "warnings": warnings,
        "config": asdict(cfg),
        "meta": dict(frame.meta(), frame_note=("Local ENU metres from the start point: x east, y north, z up "
                                              "(AGL, flat terrain). lon/lat are the exact inverse mapping."),
                     bounds_local=list(bounds), bounds_geo={"sw": list(sw), "ne": list(ne)},
                     grid_cell_m=cs, grid_layer_m=grid.dz, grid_shape=[grid.nx, grid.ny, grid.kmax]),
        "start": {"lat": lat_s, "lon": lon_s, "x": 0.0, "y": 0.0},
        "dest": {"lat": lat_d, "lon": lon_d, "x": dx, "y": dy},
        "metrics": metrics,
        "routes": [r.to_dict() for r in routes],
        "paths": {"planned": [[w["lon"], w["lat"], w["z"], w["cum_m"]] for w in route0.waypoints],
                  "actual": actual,
                  "straight": [[lon_s, lat_s, 0.0], [lon_d, lat_d, 0.0]]},
        "frames": frames, "events": events, "buildings": bl, "nofly": nf, "dynamic": dyn,
        "traces": {"obstacles": traces, "drone": actual},
        "camera": {"lat": mid[0], "lon": mid[1], "zoom": zoom, "pitch": 55.0,
                   "bearing": bearing_deg(dx, dy)},
        "buildings_source": source, "buildings_note": note,
    }
    result["validation"] = validate_result(result)
    return result


# --------------------------------------------------------------------------
# Validation: coordinate conversion, distances, interpolation consistency
# --------------------------------------------------------------------------
def validate_result(res: dict, tol=1e-6):
    """Return a list of problems (empty list = every check passed)."""
    issues = []
    m = res["meta"]
    fr = GeoFrame(m["origin_lat"], m["origin_lon"])
    if abs(fr.m_per_deg_lat - m["m_per_deg_lat"]) > 1e-9 or abs(fr.m_per_deg_lon - m["m_per_deg_lon"]) > 1e-9:
        issues.append("meta metres-per-degree does not match the geo frame")
    frames, mt = res["frames"], res["metrics"]
    for f in frames:  # lat/lon <-> x/y round trip and altitude range
        x, y = fr.to_local(f["lat"], f["lon"])
        if abs(x - f["x"]) > tol or abs(y - f["y"]) > tol:
            issues.append(f"frame t={f['t']:.1f}: lat/lon do not round-trip to x/y")
            break
    ceiling = res["config"]["ceiling_m"]
    if any(f["z"] < -1e-9 or f["z"] > ceiling + 1e-6 for f in frames):
        issues.append("a frame altitude is below ground or above the planning ceiling")
    r0 = res["routes"][0]
    if abs(sum(l["length_m"] for l in r0["legs"]) - mt["planned_distance_m"]) > tol:
        issues.append("planned leg lengths do not add up to the planned distance")
    for r in res["routes"]:
        for a, b in zip(r["legs"], r["legs"][1:]):
            if abs(a["cum_end_m"] - b["cum_start_m"]) > tol:
                issues.append(f"route v{r['ver']}: cumulative distance is not continuous")
                break
        for l in r["legs"]:
            if abs(math.sqrt(l["horiz_m"] ** 2 + l["vert_m"] ** 2) - l["length_m"]) > 1e-6:
                issues.append(f"route v{r['ver']} leg {l['idx']}: 3D length != hypot(horizontal, vertical)")
                break
    w0, w1 = r0["waypoints"][0], r0["waypoints"][-1]
    if abs(w0["lat"] - res["start"]["lat"]) > 1e-9 or abs(w0["lon"] - res["start"]["lon"]) > 1e-9 or abs(w0["z"]) > 1e-9:
        issues.append("first planned waypoint is not the start point at ground level")
    if abs(w1["lat"] - res["dest"]["lat"]) > 1e-7 or abs(w1["lon"] - res["dest"]["lon"]) > 1e-7 or abs(w1["z"]) > 1e-9:
        issues.append("last planned waypoint is not the destination at ground level")
    sl = math.hypot(res["dest"]["x"], res["dest"]["y"])
    if abs(sl - mt["straight_line_m"]) > tol:
        issues.append("straight-line metric differs from the local-frame distance")
    if abs(sl - mt["haversine_straight_m"]) / sl > 0.005:
        issues.append("local-frame straight-line distance differs from haversine by more than 0.5 %")
    # actual distance == sum of 3D steps between frames, and cumulative is monotonic
    acc = 0.0
    for a, b in zip(frames, frames[1:]):
        acc += dist3((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"]))
        if b["dist"] < a["dist"] - 1e-9 or b["t"] < a["t"] - 1e-9:
            issues.append("distance or time goes backwards between frames")
            break
    if abs(acc - mt["actual_distance_m"]) > 1e-4:
        issues.append(f"frame-to-frame distance ({acc:.4f} m) != actual distance ({mt['actual_distance_m']:.4f} m)")
    # every frame lies on its route polyline (viewer interpolation stays on the path)
    for f in frames:
        if f["state"] in ("aborted",):
            continue
        r = res["routes"][f["ver"]]
        best = 1e18
        for l in r["legs"]:
            ax, ay, az, bx, by, bz = l["x0"], l["y0"], l["z0"], l["x1"], l["y1"], l["z1"]
            ux, uy, uz = bx - ax, by - ay, bz - az
            ll = ux * ux + uy * uy + uz * uz
            u = max(0.0, min(1.0, ((f["x"] - ax) * ux + (f["y"] - ay) * uy + (f["z"] - az) * uz) / ll))
            best = min(best, dist3((f["x"], f["y"], f["z"]), (ax + ux * u, ay + uy * u, az + uz * u)))
        if best > 1e-6:
            issues.append(f"frame t={f['t']:.1f} is {best:.4f} m off its route")
            break
    for e in res["events"]:
        la, lo = fr.to_geo(e["x"], e["y"])
        if abs(la - e["lat"]) > 1e-9 or abs(lo - e["lon"]) > 1e-9:
            issues.append(f"event {e['id']} lat/lon does not match its x/y")
    if res["failure"] is None:
        last = frames[-1]
        if math.hypot(last["x"] - res["dest"]["x"], last["y"] - res["dest"]["y"]) > 0.05 or last["z"] > 0.05:
            issues.append("final frame is not at the destination on the ground")
        if abs(mt["actual_distance_m"] - mt["planned_distance_m"]) > 1e-6 and not res["events"]:
            issues.append("actual != planned distance although no re-route/hold occurred")
    return issues


# --------------------------------------------------------------------------
# Self test
# --------------------------------------------------------------------------
def _selftest():
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + name + (f"  {extra}" if extra else ""))
        ok = ok and cond

    # 1. coordinate round trip at several latitudes
    for lat in (0.0, 17.44, 51.5, 60.0):
        fr = GeoFrame(lat, 78.0)
        worst = 0.0
        for dx, dy in ((0, 0), (1234.5, -987.6), (-3000, 2500)):
            la, lo = fr.to_geo(dx, dy)
            x, y = fr.to_local(la, lo)
            worst = max(worst, abs(x - dx), abs(y - dy))
        d_loc = math.hypot(*fr.to_local(*fr.to_geo(2000, 1500)))
        hv = haversine_m(lat, 78.0, *fr.to_geo(2000, 1500))
        check(f"geo<->local round trip + distance vs haversine @ lat {lat}", worst < 1e-6 and abs(d_loc - hv) / hv < 0.005,
              f"(err {worst:.2e} m, d_local {d_loc:.2f} vs haversine {hv:.2f})")
    # 2. full simulations with synthetic buildings (offline-safe)
    scenarios = [
        ("default, no wind", Config(use_osm=False)),
        ("wind + hold response", Config(use_osm=False, wind_east_ms=5, wind_north_ms=-3, response="hold")),
        ("reroute response, altitude 35 m", Config(use_osm=False, altitude_m=35, response="reroute")),
        ("no dynamic obstacles", Config(use_osm=False, use_dynamic=False)),
        ("other area", Config(use_osm=False, start=(51.5007, -0.1246), dest=(51.5055, -0.0754))),
    ]
    for name, cfg in scenarios:
        t0 = time.time()
        res = run_simulation(cfg)
        if not res.get("ok"):
            check(f"simulate: {name}", False, res.get("error"))
            continue
        mt = res["metrics"]
        iss = res["validation"]
        check(f"simulate: {name}", not iss and mt["min_clearance_m"] > 0,
              f"planned {mt['planned_distance_m']:.1f} m | straight {mt['straight_line_m']:.1f} m | actual "
              f"{mt['actual_distance_m']:.1f} m | reroutes {mt['reroute_count']} holds {mt['hold_count']} | "
              f"frames {len(res['frames'])} | min clr {mt['min_clearance_m']:.1f} m | {time.time() - t0:.1f}s"
              + (f" | ISSUES {iss}" if iss else ""))
        if name == "no dynamic obstacles":
            check("planned == actual distance without events", abs(mt["planned_distance_m"] - mt["actual_distance_m"]) < 1e-6)
        if name == "wind + hold response":
            check("hold event produced under hold response", mt["hold_count"] >= 1, f"holds={mt['hold_count']}")
    # 3. bad input is readable
    r = run_simulation(Config(start=(17.0, 78.0), dest=(17.0, 78.0)))
    check("identical points -> readable error", (not r["ok"]) and "apart" in r["error"], r.get("error", ""))
    print("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        raise SystemExit(_selftest())
    print(__doc__)
