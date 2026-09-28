"""Milestone 3 - 3D A* on the (x, y, altitude-level) lattice with 26-neighbour moves."""
from __future__ import annotations
import heapq, math, time
from dataclasses import dataclass
import numpy as np
from ai_engine.world import World


@dataclass
class PlanResult:
    path: np.ndarray        # (n, 3) waypoints
    raw_count: int
    expanded: int
    ms: int


def smooth(w, raw):
    """String-pulling: skip waypoints whenever the straight 3D segment is collision-free."""
    path, i = [raw[0]], 0
    while i < len(raw) - 1:
        j = len(raw) - 1
        while j > i + 1 and not w.seg_free(raw[i], raw[j]):
            j -= 1
        path.append(raw[j]); i = j
    return np.array(path)


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
    return PlanResult(smooth(w, raw), len(raw), pops, int((time.time() - t0) * 1000))
