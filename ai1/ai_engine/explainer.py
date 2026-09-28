"""Milestone 6 - turn the planned geometry into human-readable \"why\" statements."""
from __future__ import annotations
import math
import numpy as np
from backend.physics import bearing, sample


def explain(path, w, res) -> list[str]:
    """`w` is a World, `res` a PlanResult (only .expanded/.ms/.raw_count are used)."""
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
