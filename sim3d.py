"""3D drone simulation engine (standard library only, no Streamlit code).

Pipeline:  real/sample buildings -> 3D grid -> 3D A* (climb, cruise, descend)
           -> step-by-step flight with dynamic re-routing -> geo-referenced result.
"""
import heapq
import itertools
import json
import math
import random
import re
import time
import urllib.parse
import urllib.request

EARTH_R = 6371000.0
Z_CELL_M = 5.0     # one vertical cell = 5 m
G = 9.81
MAX_SPAN_M = 5000  # longest supported trip


# ------------------------------------------------------------------ geometry
def latlon_to_local(lat, lon, lat0, lon0):
    dx = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    dy = math.radians(lat - lat0) * EARTH_R
    return dx, dy


def local_to_latlon(dx, dy, lat0, lon0):
    lat = lat0 + math.degrees(dy / EARTH_R)
    lon = lon0 + math.degrees(dx / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def circle_polygon(lat0, lon0, radius_m, n=40):
    pts = []
    for i in range(n + 1):
        a = 2 * math.pi * i / n
        lat, lon = local_to_latlon(radius_m * math.cos(a), radius_m * math.sin(a), lat0, lon0)
        pts.append([round(lon, 6), round(lat, 6)])
    return pts


def point_in_polygon(x, y, poly):
    inside, j = False, len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def route_bbox(start, goal, pad_m=150):
    """(south, west, north, east) around the route, padded by pad_m metres."""
    gx, gy = latlon_to_local(goal[0], goal[1], start[0], start[1])
    x0, x1 = min(0, gx) - pad_m, max(0, gx) + pad_m
    y0, y1 = min(0, gy) - pad_m, max(0, gy) + pad_m
    south, west = local_to_latlon(x0, y0, start[0], start[1])
    north, east = local_to_latlon(x1, y1, start[0], start[1])
    return south, west, north, east


# ------------------------------------------------------------------ buildings
def _parse_height(tags):
    h = tags.get("height")
    if h:
        m = re.search(r"[\d.]+", str(h))
        if m:
            value = float(m.group())
            if "ft" in str(h) or "'" in str(h):
                value *= 0.3048
            return max(3.0, value)
    levels = tags.get("building:levels")
    if levels:
        m = re.search(r"[\d.]+", str(levels))
        if m:
            return max(3.0, float(m.group()) * 3.2)
    return 12.0  # unknown: assume about 3-4 floors


def parse_osm_buildings(payload):
    """Overpass JSON ('out geom') -> [{"poly": [(lon, lat), ...], "h": metres}]."""
    out = []
    for el in payload.get("elements", []):
        geom = el.get("geometry")
        if el.get("type") != "way" or not geom or len(geom) < 4:
            continue
        poly = [(g["lon"], g["lat"]) for g in geom]
        out.append({"poly": poly, "h": _parse_height(el.get("tags", {}))})
    return out


OVERPASS = ["https://overpass-api.de/api/interpreter",
            "https://overpass.kumi.systems/api/interpreter"]


def fetch_osm_buildings(bbox, limit=1500, timeout=20):
    """Download real building footprints and heights from OpenStreetMap."""
    s, w, n, e = bbox
    query = (f'[out:json][timeout:{timeout}];'
             f'way["building"]({s:.6f},{w:.6f},{n:.6f},{e:.6f});out geom {limit};')
    body = urllib.parse.urlencode({"data": query}).encode()
    last_error = None
    for url in OVERPASS:
        try:
            req = urllib.request.Request(url, body, {"User-Agent": "drone-path-planner-demo/1.0"})
            with urllib.request.urlopen(req, timeout=timeout + 5) as r:
                return parse_osm_buildings(json.load(r))
        except Exception as error:  # try the next mirror
            last_error = error
    raise RuntimeError(f"OpenStreetMap request failed: {last_error}")


def synthetic_buildings(start, goal, count=24):
    """Sample city blocks along the route (used when real data is unavailable)."""
    gx, gy = latlon_to_local(goal[0], goal[1], start[0], start[1])
    span = math.hypot(gx, gy)
    if span < 1:
        return []
    seed = int(abs(start[0]) * 1e4 + abs(start[1]) * 1e4 + abs(goal[0]) * 1e4 + abs(goal[1]) * 1e4)
    rng = random.Random(seed)
    ux, uy = gx / span, gy / span
    px, py = -uy, ux
    out = []
    for _ in range(count):
        t = rng.uniform(0.06, 0.94)
        off = rng.choice([-1, 1]) * rng.uniform(0.01, 0.24) * span
        cx, cy = gx * t + px * off, gy * t + py * off
        w, d = rng.uniform(25, 80), rng.uniform(25, 80)
        a = rng.uniform(0, math.pi / 2)
        corners = []
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1)):
            lx, ly = sx * w / 2, sy * d / 2
            x = cx + lx * math.cos(a) - ly * math.sin(a)
            y = cy + lx * math.sin(a) + ly * math.cos(a)
            lat, lon = local_to_latlon(x, y, start[0], start[1])
            corners.append((lon, lat))
        out.append({"poly": corners,
                    "h": rng.choice([18, 25, 35, 50, 70, 95, 120, 150]) * rng.uniform(0.8, 1.2)})
    return out


# -------------------------------------------------------------------- physics
class Physics:
    """Simple electric-multirotor energy model (joules)."""

    def __init__(self, base_kg, payload_kg, speed=10.0):
        self.mass = base_kg + payload_kg
        self.v = speed
        self.hover_w = 110.0 * self.mass  # electrical power needed to stay aloft
        self.cda = 0.06                   # drag area, m^2
        self.rho = 1.225
        self.eta = 0.6                    # climb efficiency

    def move(self, dx, dy, dz, wind):
        """Energy (J) and time (s) to fly a displacement (metres) in the given wind."""
        d = math.sqrt(dx * dx + dy * dy + dz * dz)
        if d == 0:
            return 0.0, 0.0
        t = d / self.v
        vx, vy, vz = dx / t - wind[0], dy / t - wind[1], dz / t - wind[2]
        v_air = math.sqrt(vx * vx + vy * vy + vz * vz)
        drag_w = 0.5 * self.rho * self.cda * v_air ** 3
        climb_j = self.mass * G * max(dz, 0.0) / self.eta
        return (self.hover_w + drag_w) * t + climb_j, t


# ---------------------------------------------------------------- environment
class Obstacle:
    def __init__(self, pos, vel, radius_m, kind="moving", active=True):
        self.pos = [float(p) for p in pos]   # grid coordinates
        self.vel = [float(v) for v in vel]   # cells per step
        self.radius_m = radius_m
        self.kind = kind
        self.active = active

    def advance(self, bounds):
        for k in (0, 1):
            self.pos[k] += self.vel[k]
            if self.pos[k] < 0 or self.pos[k] > bounds[k] - 1:
                self.vel[k] = -self.vel[k]
                self.pos[k] = min(max(self.pos[k], 0), bounds[k] - 1)

    def future(self, k):
        return [self.pos[i] + self.vel[i] * k for i in range(3)]


class Airspace:
    def __init__(self, bounds, cell_m, wind):
        self.bounds, self.cell_m, self.wind = bounds, cell_m, wind
        self.static, self.near, self.dynamic = set(), set(), []
        self._spheres = {}

    def finalize(self):
        """Cells next to a building: flying there is allowed but costs extra risk."""
        bx, by, bz = self.bounds
        self.near = set()
        for (x, y, z) in self.static:
            for dx, dy, dz in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
                n = (x + dx, y + dy, z + dz)
                if 0 <= n[0] < bx and 0 <= n[1] < by and 0 <= n[2] < bz and n not in self.static:
                    self.near.add(n)

    def dist_m(self, cell, pos):
        return math.sqrt(((cell[0] - pos[0]) * self.cell_m) ** 2
                         + ((cell[1] - pos[1]) * self.cell_m) ** 2
                         + ((cell[2] - pos[2]) * Z_CELL_M) ** 2)

    def clearance_m(self, cell):
        best = math.inf
        for o in self.dynamic:
            if o.active:
                best = min(best, self.dist_m(cell, o.pos) - o.radius_m)
        return best

    def _sphere(self, radius_m):
        key = round(radius_m)
        if key not in self._spheres:
            rx = int(radius_m / self.cell_m) + 1
            rz = int(radius_m / Z_CELL_M) + 1
            self._spheres[key] = [
                (a, b, c) for a in range(-rx, rx + 1) for b in range(-rx, rx + 1)
                for c in range(-rz, rz + 1)
                if (a * self.cell_m) ** 2 + (b * self.cell_m) ** 2 + (c * Z_CELL_M) ** 2 <= radius_m ** 2
            ]
        return self._spheres[key]

    def danger_cells(self, horizon):
        """Cells around every active obstacle now and `horizon` steps ahead (+1 cell margin)."""
        cells = set()
        for o in self.dynamic:
            if not o.active:
                continue
            offsets = self._sphere(o.radius_m + self.cell_m)
            for k in range(horizon + 1):
                fx, fy, fz = o.future(k)
                cx, cy, cz = round(fx), round(fy), round(fz)
                for a, b, c in offsets:
                    cells.add((cx + a, cy + b, cz + c))
        return cells


# -------------------------------------------------------------------- planner
class Planner:
    """A* over a 26-neighbour 3D grid with a multi-factor cost (distance, energy, risk)."""

    def __init__(self, env, physics, w_dist, w_energy, w_risk, max_expansions=250000):
        self.env, self.w_dist, self.max_expansions = env, w_dist, max_expansions
        e_ref = physics.hover_w / physics.v  # joules per metre of plain flight
        self.dirs = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == dy == dz == 0:
                        continue
                    mx, my, mz = dx * env.cell_m, dy * env.cell_m, dz * Z_CELL_M
                    d = math.sqrt(mx * mx + my * my + mz * mz)
                    e, _ = physics.move(mx, my, mz, env.wind)
                    base = w_dist * d + w_energy * e / e_ref
                    axes = [(dx, 0, 0), (0, dy, 0), (0, 0, dz)]
                    axes = [a for a in axes if any(a)]
                    subs = []  # diagonal moves may not squeeze past a corner
                    for r in range(1, len(axes) + 1):
                        for combo in itertools.combinations(axes, r):
                            s = tuple(sum(v) for v in zip(*combo))
                            if s != (dx, dy, dz):
                                subs.append(s)
                    self.dirs.append((dx, dy, dz, base, w_risk * 0.5 * d, subs))
        self.expanded = 0

    def astar(self, start, goal, extra=frozenset()):
        env = self.env
        static, near = env.static, env.near
        bx, by, bz = env.bounds
        cm = env.cell_m
        if goal in static or goal in extra:
            return None
        wd = self.w_dist

        def h(c):
            return wd * math.sqrt(((c[0] - goal[0]) * cm) ** 2 + ((c[1] - goal[1]) * cm) ** 2
                                  + ((c[2] - goal[2]) * Z_CELL_M) ** 2)

        count = itertools.count()
        heap = [(h(start), next(count), 0.0, start)]
        g, parent, closed = {start: 0.0}, {}, set()
        while heap:
            _, _, gc, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            closed.add(cur)
            if cur == goal:
                path = [cur]
                while cur in parent:
                    cur = parent[cur]
                    path.append(cur)
                self.expanded += len(closed)
                return path[::-1]
            if len(closed) > self.max_expansions:
                self.expanded += len(closed)
                return None
            x, y, z = cur
            for dx, dy, dz, base, risk_add, subs in self.dirs:
                nx, ny, nz = x + dx, y + dy, z + dz
                if nx < 0 or ny < 0 or nz < 0 or nx >= bx or ny >= by or nz >= bz:
                    continue
                n = (nx, ny, nz)
                if n in static or n in extra:
                    continue
                if any(((x + a, y + b, z + c) in static or (x + a, y + b, z + c) in extra)
                       for a, b, c in subs):
                    continue
                ng = gc + base + (risk_add if n in near else 0.0)
                if ng < g.get(n, math.inf):
                    g[n] = ng
                    parent[n] = cur
                    heapq.heappush(heap, (ng + h(n), next(count), ng, n))
        self.expanded += len(closed)
        return None

    def plan(self, start, goal, cruise_z, extra=frozenset()):
        """Climb -> cruise -> descend. Falls back to a direct plan if a leg fails."""
        top = max(cruise_z, start[2], goal[2])
        up, down = (start[0], start[1], top), (goal[0], goal[1], top)
        leg1 = self.astar(start, up, extra)
        leg2 = self.astar(up, down, extra) if leg1 else None
        leg3 = self.astar(down, goal, extra) if leg2 else None
        if leg1 and leg2 and leg3:
            return leg1 + leg2[1:] + leg3[1:]
        return self.astar(start, goal, extra)


# ----------------------------------------------------------------- simulation
def _rasterize(env, buildings, lat0, lon0, ox, oy):
    """Turn building footprints into blocked 3D cells. Returns the buildings kept."""
    bx, by, bz = env.bounds
    kept = []
    for b in buildings[:1500]:
        pts = [latlon_to_local(lat, lon, lat0, lon0) for lon, lat in b["poly"]]
        xs = [p[0] / env.cell_m + ox for p in pts]
        ys = [p[1] / env.cell_m + oy for p in pts]
        if max(xs) < 0 or min(xs) > bx - 1 or max(ys) < 0 or min(ys) > by - 1:
            continue
        poly = list(zip(xs, ys))
        x0, x1 = max(0, math.floor(min(xs))), min(bx - 1, math.ceil(max(xs)))
        y0, y1 = max(0, math.floor(min(ys))), min(by - 1, math.ceil(max(ys)))
        cells = [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)
                 if point_in_polygon(x, y, poly)]
        if not cells:  # smaller than one cell: block the cell it sits in
            cx, cy = round(sum(xs) / len(xs)), round(sum(ys) / len(ys))
            if 0 <= cx < bx and 0 <= cy < by:
                cells = [(cx, cy)]
        top = min(bz, max(1, math.ceil(b["h"] / Z_CELL_M)))
        for (x, y) in cells:
            for z in range(top):
                env.static.add((x, y, z))
        kept.append({"p": [[round(lon, 6), round(lat, 6)] for lon, lat in b["poly"]],
                     "h": round(b["h"], 1)})
    return kept


def run_simulation(cfg):
    """Run a full mission. Raises ValueError with a readable message on bad input."""
    start, goal = cfg["start"], cfg["goal"]
    lat0, lon0 = start
    dx, dy = latlon_to_local(goal[0], goal[1], lat0, lon0)
    span = math.hypot(dx, dy)
    if span < 20:
        raise ValueError("Start and goal are too close together. Pick points at least 20 m apart.")
    if span > MAX_SPAN_M:
        raise ValueError(f"Start and goal are {span / 1000:.1f} km apart. The limit is {MAX_SPAN_M // 1000} km.")

    cell_m = min(max(span / 50.0, 6.0), 100.0)
    margin = 8
    gx, gy = round(dx / cell_m), round(dy / cell_m)
    ox, oy = margin + max(0, -gx), margin + max(0, -gy)
    zs, zg = round(cfg["start_alt"] / Z_CELL_M), round(cfg["goal_alt"] / Z_CELL_M)
    zc = round(cfg["cruise_alt"] / Z_CELL_M)
    bounds = (abs(gx) + 2 * margin + 1, abs(gy) + 2 * margin + 1, max(zs, zg, zc) + 8)
    start_c, goal_c = (ox, oy, zs), (ox + gx, oy + gy, zg)

    env = Airspace(bounds, cell_m, tuple(cfg["wind"]))
    physics = Physics(cfg["base_kg"], cfg["payload_kg"], cfg["speed"])
    buildings_kept, nofly = [], None

    if cfg["add_obstacles"]:
        buildings_kept = _rasterize(env, cfg["buildings"], lat0, lon0, ox, oy)
        # No-fly cylinder on the route, full height.
        cx, cy = round(start_c[0] + (goal_c[0] - start_c[0]) * 0.55), round(start_c[1] + (goal_c[1] - start_c[1]) * 0.55)
        r_m = min(max(span * 0.06, 40.0), 150.0)
        rc = r_m / cell_m
        for x in range(int(cx - rc) - 1, int(cx + rc) + 2):
            for y in range(int(cy - rc) - 1, int(cy + rc) + 2):
                if (x - cx) ** 2 + (y - cy) ** 2 <= rc ** 2 and 0 <= x < bounds[0] and 0 <= y < bounds[1]:
                    for z in range(bounds[2]):
                        env.static.add((x, y, z))
        c_lat, c_lon = local_to_latlon((cx - ox) * cell_m, (cy - oy) * cell_m, lat0, lon0)
        nofly = {"p": circle_polygon(c_lat, c_lon, r_m), "h": bounds[2] * Z_CELL_M}

        # Keep the take-off and landing pads clear.
        for (px, py, _) in (start_c, goal_c):
            for x in range(px - 1, px + 2):
                for y in range(py - 1, py + 2):
                    for z in range(bounds[2]):
                        env.static.discard((x, y, z))

        # Two moving obstacles that cross the route at cruise altitude.
        span_c = math.hypot(goal_c[0] - start_c[0], goal_c[1] - start_c[1])
        ux, uy = (goal_c[0] - start_c[0]) / span_c, (goal_c[1] - start_c[1]) / span_c
        perp = (-uy, ux)
        for t, side in ((0.33, 1), (0.72, -1)):
            off = side * 0.14 * span_c
            pos = (start_c[0] + (goal_c[0] - start_c[0]) * t + perp[0] * off,
                   start_c[1] + (goal_c[1] - start_c[1]) * t + perp[1] * off, max(zc, 2))
            vel = (-side * perp[0] * 0.35, -side * perp[1] * 0.35, 0)
            env.dynamic.append(Obstacle(pos, vel, radius_m=15.0))
    env.finalize()

    planner = Planner(env, physics, cfg["w_dist"], cfg["w_energy"], cfg["w_risk"])
    t0 = time.perf_counter()
    macro = planner.plan(start_c, goal_c, zc)
    planner_ms = (time.perf_counter() - t0) * 1000
    if not macro:
        raise ValueError("No valid flight path exists. Try a higher cruise altitude, "
                         "different points, or turn off the obstacles.")

    # ---------------------------------------------------------- flight loop
    detect = int(cfg["detect_cells"])
    path, i, pos = list(macro), 1, start_c
    dyn_pos = [[list(o.pos)] for o in env.dynamic]
    appear = [0] * len(env.dynamic)
    frames = [pos]
    events, reroutes = [0], []
    cum_e = cum_d = cum_t = 0.0
    trace = [{"step": 0, "alt_m": pos[2] * Z_CELL_M, "cum_energy_wh": 0.0,
              "cum_dist_m": 0.0, "risk": 0.0}]
    holds = collisions = consecutive_holds = 0
    min_clear = math.inf
    injected = not cfg["inject_surprise"]
    step, limit = 0, 6 * len(macro) + 200
    influence = max(60.0, 4 * cell_m)

    while pos != goal_c and step < limit:
        step += 1
        for o in env.dynamic:
            o.advance(bounds)

        if not injected and i >= len(path) // 2:
            injected = True
            radius = 25.0
            idx = min(i + max(4, detect // 2 + 2), len(path) - 1)
            while idx > i and env.dist_m(path[idx], goal_c) < 3 * (radius + cell_m):
                idx -= 1
            if idx > i:
                env.dynamic.append(Obstacle(path[idx], (0, 0, 0), radius, kind="surprise"))
                dyn_pos.append([list(path[idx]) for _ in range(step)])
                appear.append(step)

        event, nxt = 0, pos
        if cfg["reroute"]:
            danger = env.danger_cells(3)
            if any(c in danger for c in path[i:i + detect]):
                new = planner.plan(pos, goal_c, max(zc, pos[2]), frozenset(danger))
                if new and len(new) > 1:
                    path, i, event = new, 1, 1
                    reroutes.append({"f": step, "pos": pos})
                else:
                    event = 2
            if event != 2 and path[i] in env.danger_cells(1):
                event = 2
        if event != 2:
            nxt = path[i]

        if nxt == pos:
            holds += 1
            consecutive_holds += 1
            e, t = physics.hover_w * cell_m / physics.v, cell_m / physics.v
            if consecutive_holds > 80:
                break
        else:
            consecutive_holds = 0
            mx, my, mz = (nxt[0] - pos[0]) * cell_m, (nxt[1] - pos[1]) * cell_m, (nxt[2] - pos[2]) * Z_CELL_M
            e, t = physics.move(mx, my, mz, env.wind)
            cum_d += math.sqrt(mx * mx + my * my + mz * mz)
        cum_e += e
        cum_t += t
        pos = nxt
        if i < len(path) and pos == path[i]:
            i += 1

        clear = env.clearance_m(pos)
        min_clear = min(min_clear, clear)
        if clear < 0:
            collisions += 1
        risk = 0.0 if clear == math.inf else max(0.0, min(1.0, 1 - clear / influence))
        frames.append(pos)
        events.append(event)
        for k, o in enumerate(env.dynamic):
            if k < len(dyn_pos):
                dyn_pos[k].append(list(o.pos))
        trace.append({"step": step, "alt_m": pos[2] * Z_CELL_M,
                      "cum_energy_wh": cum_e / 3600.0, "cum_dist_m": cum_d, "risk": risk})

    n_frames = len(frames)
    for k in range(len(dyn_pos)):  # make every obstacle track the full flight
        while len(dyn_pos[k]) < n_frames:
            dyn_pos[k].append(dyn_pos[k][-1])

    # ---------------------------------------------------- geo-referencing
    def local_m(c):
        return (c[0] - ox) * cell_m, (c[1] - oy) * cell_m

    def geo(c):
        lat, lon = local_to_latlon(*local_m(c), lat0, lon0)
        return [round(lon, 6), round(lat, 6), round(c[2] * Z_CELL_M, 1)]

    planned_len = sum(math.dist((a[0] * cell_m, a[1] * cell_m, a[2] * Z_CELL_M),
                                (b[0] * cell_m, b[1] * cell_m, b[2] * Z_CELL_M))
                      for a, b in zip(macro, macro[1:]))
    straight = math.sqrt(span ** 2 + ((goal_c[2] - start_c[2]) * Z_CELL_M) ** 2)
    success = pos == goal_c
    metrics = {
        "success": success,
        "distance_m": cum_d,
        "planned_m": planned_len,
        "straight_m": straight,
        "efficiency": straight / cum_d if cum_d else 0.0,
        "energy_wh": cum_e / 3600.0,
        "battery_left_pct": max(0.0, 100.0 * (1 - (cum_e / 3600.0) / cfg["battery_wh"])),
        "time_s": cum_t,
        "planner_ms": planner_ms,
        "expanded": planner.expanded,
        "waypoints": len(macro),
        "max_alt_m": max(f[2] for f in frames) * Z_CELL_M,
        "reroutes": len(reroutes),
        "holds": holds,
        "collisions": collisions,
        "min_clearance_m": None if min_clear == math.inf else min_clear,
        "cell_m": cell_m,
        "span_m": span,
    }

    mid_lat, mid_lon = local_to_latlon(dx / 2, dy / 2, lat0, lon0)
    zoom = math.log2(78271.517 * math.cos(math.radians(mid_lat)) / (span * 1.7 / 700)) - 0.3
    heading = math.degrees(math.atan2(dx, dy))

    def local_row(c):
        x, y = local_m(c)
        return {"east_m": round(x, 1), "north_m": round(y, 1), "alt_m": c[2] * Z_CELL_M}

    return {
        "metrics": metrics,
        "frames": [geo(c) for c in frames],
        "planned": [geo(c) for c in macro],
        "buildings": buildings_kept,
        "nofly": nofly,
        "dyn": [{"r": o.radius_m, "kind": o.kind, "appear": appear[k],
                 "pos": [geo([p[0], p[1], p[2]]) for p in dyn_pos[k]]}
                for k, o in enumerate(env.dynamic)],
        "events": events,
        "reroute_marks": [{"f": r["f"], "pos": geo(r["pos"])} for r in reroutes],
        "trace": trace,
        "planned_local": [local_row(c) for c in macro],
        "actual_local": [local_row(c) for c in frames],
        "start": [lat0, lon0], "goal": list(goal),
        "start_alt": cfg["start_alt"], "goal_alt": cfg["goal_alt"],
        "view": {"center": [round(mid_lon, 6), round(mid_lat, 6)], "zoom": round(min(max(zoom, 10), 19), 2),
                 "bearing": round(heading - 20, 1), "pitch": 62},
        "source": cfg.get("source", "Sample"),
    }
