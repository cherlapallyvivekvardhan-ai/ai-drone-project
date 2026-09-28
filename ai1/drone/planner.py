"""3D path planning: collision world, A*, metrics, explainer. Pure numpy/stdlib - no UI code.

Local frame: x = east (m), y = north (m), z = altitude above ground (m).
"""
from __future__ import annotations
import heapq, math, time
from dataclasses import dataclass
import numpy as np

ALTS = (0, 30, 50, 70, 90, 110, 130, 150)          # altitude levels of the search lattice
ZONE_NAMES = ("Alpha", "Bravo", "Charlie")


@dataclass
class Zone:
    name: str
    x: float
    y: float
    r: float


def bearing(a, b) -> float:
    return (math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) + 360) % 360


def make_zones(goal_xy, count=3) -> list[Zone]:
    """Cylinders placed on/near the straight A->B line so the drone must bypass them."""
    L = math.hypot(*goal_xy)
    ux, uy = goal_xy[0] / L, goal_xy[1] / L
    r = min(max(60.0, L * 0.08), L * 0.14)
    spec = [(.30, 0.0), (.58, .7), (.80, -.6)][:count]
    return [Zone(ZONE_NAMES[i], ux * L * t - uy * r * off, uy * L * t + ux * r * off, r) for i, (t, off) in enumerate(spec)]


def _inside(px, py, ring):
    """Vectorised ray-casting point-in-polygon."""
    ins = np.zeros(px.shape, bool)
    x, y = ring[:, 0], ring[:, 1]
    for i in range(len(ring)):
        xi, yi, xj, yj = x[i], y[i], x[i - 1], y[i - 1]
        ins ^= ((yi > py) != (yj > py)) & (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi)
    return ins


class World:
    """Milestone 2 - collision model. Buildings -> height raster; zones -> infinite cylinders.
    A point is flyable iff it is above roof + clearance and outside every zone (pads are exempt)."""

    def __init__(self, bounds, cell, clearance, buildings, zones, pads, alts=ALTS):
        self.minx, self.miny, self.maxx, self.maxy = bounds
        self.cell, self.clearance, self.zones, self.pads, self.alts = cell, clearance, zones, pads, alts
        self.nx = math.ceil((self.maxx - self.minx) / cell)
        self.ny = math.ceil((self.maxy - self.miny) / cell)
        self.H = np.zeros((self.ny, self.nx), np.float32)
        self.n_buildings = len(buildings)
        for ring, h in buildings:
            self._raster(np.asarray(ring, float), h)
        self._build_blocked()

    def ci(self, x): return int(min(self.nx - 1, max(0, (x - self.minx) // self.cell)))
    def cj(self, y): return int(min(self.ny - 1, max(0, (y - self.miny) // self.cell)))

    def _raster(self, ring, h):
        x0, y0 = ring.min(0); x1, y1 = ring.max(0)
        if x1 < self.minx or x0 > self.maxx or y1 < self.miny or y0 > self.maxy:
            return
        for x, y in ring:                                    # tiny buildings still mark their cell
            j, i = self.cj(y), self.ci(x); self.H[j, i] = max(self.H[j, i], h)
        i0, i1, j0, j1 = self.ci(x0), self.ci(x1), self.cj(y0), self.cj(y1)
        X, Y = np.meshgrid(self.minx + (np.arange(i0, i1 + 1) + .5) * self.cell,
                           self.miny + (np.arange(j0, j1 + 1) + .5) * self.cell)
        sub = self.H[j0:j1 + 1, i0:i1 + 1]
        m = _inside(X, Y, ring)
        sub[m] = np.maximum(sub[m], h)

    def _build_blocked(self):
        X, Y = np.meshgrid(self.minx + (np.arange(self.nx) + .5) * self.cell, self.miny + (np.arange(self.ny) + .5) * self.cell)
        zone = np.zeros_like(X, bool)
        for z in self.zones:
            zone |= np.hypot(X - z.x, Y - z.y) < z.r + 12
        alts = np.array(self.alts, float)[:, None, None]
        blocked = (alts < self.H[None] + self.clearance) | zone[None]
        blocked[0] = True                                    # ground level exists only on pads
        for px, py in self.pads:
            blocked[:, self.cj(py), self.ci(px)] = False
        self.blocked = blocked.ravel().tolist()

    def is_pad(self, x, y): return any(self.ci(x) == self.ci(p[0]) and self.cj(y) == self.cj(p[1]) for p in self.pads)
    def h_at(self, x, y): return float(self.H[self.cj(y), self.ci(x)])
    def h_near(self, x, y):
        j, i = self.cj(y), self.ci(x)
        return float(self.H[max(0, j - 1):j + 2, max(0, i - 1):i + 2].max())

    def free(self, x, y, z) -> bool:
        if not (self.minx <= x <= self.maxx and self.miny <= y <= self.maxy):
            return False
        if self.is_pad(x, y):
            return True
        return all(math.hypot(x - q.x, y - q.y) >= q.r + 12 for q in self.zones) and z >= self.h_at(x, y) + self.clearance

    def seg_free(self, a, b) -> bool:
        n = max(1, math.ceil(math.dist(a, b) / (self.cell / 5)))
        return all(self.free(*(a[k] + (b[k] - a[k]) * t / n for k in range(3))) for t in range(n + 1))


@dataclass
class PlanResult:
    path: np.ndarray        # (n, 3) waypoints
    raw_count: int
    expanded: int
    ms: int


def plan(w: World, start, goal) -> PlanResult | None:
    """Milestone 3 - 3D A* on the (x, y, altitude-level) lattice with 26-neighbour moves."""
    t0 = time.time()
    nx, ny, K, cell, alts = w.nx, w.ny, len(w.alts), w.cell, w.alts
    idx = lambda i, j, k: (k * ny + j) * nx + i
    Hf = w.H.ravel().tolist()
    si, sj, gi, gj = w.ci(start[0]), w.cj(start[1]), w.ci(goal[0]), w.cj(goal[1])
    s, e = idx(si, sj, 0), idx(gi, gj, 0)
    gx, gy = w.minx + (gi + .5) * cell, w.miny + (gj + .5) * cell
    padcells = {(w.ci(p[0]), w.cj(p[1])) for p in w.pads}
    moves = [(di, dj, dk) for di in (-1, 0, 1) for dj in (-1, 0, 1) for dk in (-1, 0, 1) if di or dj or dk]
    g, came, closed, heap, pops = {s: 0.0}, {}, set(), [(0.0, s)], 0
    while heap and pops < 600_000:
        _, c = heapq.heappop(heap)
        if c in closed:
            continue
        closed.add(c); pops += 1
        if c == e:
            break
        i, j, k = c % nx, (c // nx) % ny, c // (nx * ny)
        for di, dj, dk in moves:
            ni, nj, nk = i + di, j + dj, k + dk
            if not (0 <= ni < nx and 0 <= nj < ny and 0 <= nk < K):
                continue
            n = idx(ni, nj, nk)
            if n in closed or w.blocked[n]:
                continue
            if di and dj and (w.blocked[idx(i + di, j, min(k, nk))] or w.blocked[idx(i, j + dj, min(k, nk))]):
                continue                                     # no cutting building corners diagonally
            if (di or dj) and ((ni, nj) in padcells or (i, j) in padcells):   # pad cells skip the grid test,
                pa = (w.minx + (i + .5) * cell, w.miny + (j + .5) * cell, alts[k])         # so check this edge continuously
                pb = (w.minx + (ni + .5) * cell, w.miny + (nj + .5) * cell, alts[nk])
                if not w.seg_free(pa, pb):
                    continue
            cost = math.sqrt((di * cell) ** 2 + (dj * cell) ** 2 + (alts[nk] - alts[k]) ** 2) * (1.15 if dk else 1.0)
            if alts[nk] - Hf[nj * nx + ni] < w.clearance + 15 and nk > 0:
                cost *= 1.3                                  # prefer a wider margin over roofs
            ng = g[c] + cost
            if ng < g.get(n, math.inf):
                g[n], came[n] = ng, c
                hx, hy = w.minx + (ni + .5) * cell, w.miny + (nj + .5) * cell
                heapq.heappush(heap, (ng + math.sqrt((hx - gx) ** 2 + (hy - gy) ** 2 + alts[nk] ** 2), n))
    if e not in came and s != e:
        return None
    raw, c = [], e
    while True:
        raw.append((w.minx + (c % nx + .5) * cell, w.miny + ((c // nx) % ny + .5) * cell, float(alts[c // (nx * ny)])))
        if c == s:
            break
        c = came[c]
    raw.reverse()            # endpoints are the pad-cell centres (<= cell/2 from the requested points)
    path, i = [raw[0]], 0                                    # string-pulling smoother
    while i < len(raw) - 1:
        j = len(raw) - 1
        while j > i + 1 and not w.seg_free(raw[i], raw[j]):
            j -= 1
        path.append(raw[j]); i = j
    return PlanResult(np.array(path), len(raw), pops, int((time.time() - t0) * 1000))


def sample(path, step):
    """Resample every `step` m -> (n, 5) array [x, y, z, heading_x, heading_y] for ribbons/animation."""
    out, hx, hy = [], *(path[-1][:2] - path[0][:2])
    for a, b in zip(path[:-1], path[1:]):
        if math.hypot(*(b[:2] - a[:2])) > 1:
            hx, hy = b[:2] - a[:2]
        d = math.hypot(hx, hy) or 1
        n = max(1, math.ceil(np.linalg.norm(b - a) / step))
        out += [[*(a + (b - a) * t / n), hx / d, hy / d] for t in range(n)]
    out.append([*path[-1], out[-1][3], out[-1][4]])
    return np.array(out)


def metrics(path, w: World, speed=12.0) -> dict:
    """Milestone 6a - dashboard numbers (simulated battery: 0.012 %/m + 0.06 %/m climbed)."""
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    climb = np.clip(np.diff(path[:, 2]), 0, None).sum()
    pts = sample(path, 10)
    vert = [p[2] - w.h_at(p[0], p[1]) for p in pts if p[2] >= 25 and not w.is_pad(p[0], p[1])] or [99]
    nfz = [math.hypot(p[0] - z.x, p[1] - z.y) - z.r for p in pts for z in w.zones] or [999]
    margin = min((min(vert) - w.clearance) / 20, (min(nfz) - 12) / 40)        # 0 = legal minimum, 1 = generous
    score = round(max(0, min(100, 60 + 40 * margin)))
    return dict(dist=float(seg.sum()), seconds=float(seg.sum() / speed), battery=float(seg.sum() * .012 + climb * .06),
                min_vert=float(min(vert)), min_nfz=float(min(nfz)), score=score)


def explain(path, w: World, res: PlanResult) -> list[str]:
    """Milestone 6b - turn the geometry into "why" statements."""
    D, direct, log = path[-1], bearing(path[0], path[-1]), []
    side = lambda d: "right" if d > 0 else "left"
    hit = [z.name for z in w.zones if abs(D[0] * (z.y) - D[1] * z.x) / (math.hypot(D[0], D[1])) < z.r]
    tall = max((w.h_near(p[0], p[1]) for p in sample(np.array([path[0], D]), 15)), default=0)
    log.append(f"Scanned {w.n_buildings} buildings. The straight line crosses "
               f"{('Zone ' + ', '.join(hit)) if hit else 'no restricted zones'} and buildings up to {tall:.0f} m, so it was rejected.")
    for i, (a, b) in enumerate(zip(path[:-1], path[1:])):
        n, dz, dh = f"Leg {i + 1}: ", b[2] - a[2], math.hypot(*(b[:2] - a[:2]))
        obs = max(w.h_near(p[0], p[1]) for p in sample(path[i:i + 3], 15))
        if dz >= 8:
            log.append(f"{n}Ascended to {b[2]:.0f} m to clear local obstructions (tallest nearby roof {obs:.0f} m + {w.clearance:.0f} m safety buffer).")
        elif dz <= -8:
            log.append(f"{n}Descended safely onto the destination pad." if b[2] < 1 else f"{n}Descended to {b[2]:.0f} m as roofs ahead got lower.")
        else:
            dev = (bearing(a, b) - direct + 540) % 360 - 180
            best, nd = None, 1e9
            for z in w.zones:
                for p, q in ((a, b), (b, path[min(i + 2, len(path) - 1)])):
                    v = q[:2] - p[:2]; t = np.clip(np.dot(np.array([z.x, z.y]) - p[:2], v) / (v @ v or 1), 0, 1)
                    d = math.hypot(z.x - p[0] - v[0] * t, z.y - p[1] - v[1] * t) - z.r
                    if d < nd: best, nd = z, d
            if abs(dev) > 8 and nd < 150:
                log.append(f"{n}Diverted {abs(dev):.0f}° {side(dev)} of the direct course (heading {bearing(a, b):.0f}°) to bypass Restricted Airspace Zone {best.name}.")
            elif abs(dev) > 8:
                log.append(f"{n}Steered {abs(dev):.0f}° {side(dev)} (heading {bearing(a, b):.0f}°) to stay clear of building masses.")
            elif nd < 40:
                log.append(f"{n}Cruised {dh:.0f} m at {a[2]:.0f} m while skirting the edge of Restricted Airspace Zone {best.name}.")
            else:
                log.append(f"{n}Cruised {dh:.0f} m at {a[2]:.0f} m, close to the direct course.")
    log.append(f"Search: 3D A* expanded {res.expanded:,} nodes in {res.ms} ms; {res.raw_count} grid steps were smoothed into {len(path)} waypoints.")
    return log
