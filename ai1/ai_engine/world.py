"""Milestone 2 - 3D collision world (buildings, no-fly cylinders, roof-clearance rule).

Local frame: x = east (m), y = north (m), z = altitude above ground (m).
"""
from __future__ import annotations
import math
from dataclasses import dataclass
import numpy as np

ALTS = (0, 30, 50, 70, 90, 110, 130, 150)          # altitude levels of the search lattice


@dataclass
class Zone:
    name: str
    x: float
    y: float
    r: float


DEFAULT_LAYOUT = [{"name": "Alpha", "t": .30, "offset": 0.0}, {"name": "Bravo", "t": .58, "offset": .7}, {"name": "Charlie", "t": .80, "offset": -.6}]


def make_zones(goal_xy, count=3, layout=None) -> list[Zone]:
    """Cylinders placed on/near the straight A->B line so the drone must bypass them."""
    L = math.hypot(*goal_xy)
    ux, uy = goal_xy[0] / L, goal_xy[1] / L
    r = min(max(60.0, L * 0.08), L * 0.14)
    return [Zone(s["name"], ux * L * s["t"] - uy * r * s["offset"], uy * L * s["t"] + ux * r * s["offset"], r)
            for s in (layout or DEFAULT_LAYOUT)[:count]]


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

    def blocked_cells(self, k):
        """Blocked (x, y) cells at altitude level k, in the {"x","y"} format of the original 2D engine."""
        b = np.array(self.blocked).reshape(len(self.alts), self.ny, self.nx)[k]
        return [{"x": int(i), "y": int(j)} for j, i in zip(*np.nonzero(b))]
