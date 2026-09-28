"""Geo helpers: local metric frame, geocoding (Nominatim), buildings (OSM Overpass), demo city."""
from __future__ import annotations
import math, re
import numpy as np
import requests

R = 6378137.0
UA = {"User-Agent": "ai-drone-path-planner/1.0 (learning project)"}


def to_local(origin, lng, lat):
    """lng/lat -> metres east/north of `origin` (lng, lat). Works on arrays."""
    return ((np.asarray(lng) - origin[0]) * math.pi / 180 * R * math.cos(math.radians(origin[1])),
            (np.asarray(lat) - origin[1]) * math.pi / 180 * R)


def to_lnglat(origin, x, y):
    return (origin[0] + np.asarray(x) / (R * math.cos(math.radians(origin[1]))) * 180 / math.pi,
            origin[1] + np.asarray(y) / R * 180 / math.pi)


def geocode(query: str) -> tuple[float, float]:
    r = requests.get("https://nominatim.openstreetmap.org/search", headers=UA, timeout=15,
                     params={"q": query, "format": "json", "limit": 1})
    r.raise_for_status()
    if not r.json():
        raise ValueError(f"Address not found: {query!r}")
    return float(r.json()[0]["lon"]), float(r.json()[0]["lat"])


def _height(tags: dict) -> float:
    m = re.match(r"[\d.]+", str(tags.get("height", "")))
    if m:
        return float(m.group())
    if "building:levels" in tags:
        try:
            return float(tags["building:levels"]) * 3.2
        except ValueError:
            pass
    return 10.0


def fetch_buildings(origin, bounds):
    """OSM building footprints inside local `bounds` -> [(ring Nx2 local metres, height m)]."""
    minx, miny, maxx, maxy = bounds
    (w, s), (e, n) = to_lnglat(origin, minx, miny), to_lnglat(origin, maxx, maxy)
    q = f'[out:json][timeout:25];way["building"]({s:.6f},{w:.6f},{n:.6f},{e:.6f});out geom;'
    r = requests.post("https://overpass-api.de/api/interpreter", data={"data": q}, headers=UA, timeout=40)
    r.raise_for_status()
    out = []
    for el in r.json().get("elements", []):
        g = el.get("geometry")
        if g and len(g) >= 3:
            x, y = to_local(origin, [p["lon"] for p in g], [p["lat"] for p in g])
            out.append((np.column_stack([x, y]), _height(el.get("tags", {}))))
    return out


def demo_buildings(bounds, keep_clear, seed=7, count=140):
    """Offline synthetic city so the app runs without internet. `keep_clear`: [(x, y)] pads."""
    rng = np.random.default_rng(seed)
    minx, miny, maxx, maxy = bounds
    out = []
    for _ in range(count):
        cx, cy = rng.uniform(minx, maxx), rng.uniform(miny, maxy)
        if any(math.hypot(cx - px, cy - py) < 60 for px, py in keep_clear):
            continue
        w, d = rng.uniform(20, 60, 2)
        out.append((np.array([[cx - w, cy - d], [cx + w, cy - d], [cx + w, cy + d], [cx - w, cy + d]]),
                    float(rng.choice([12, 18, 25, 40, 60, 90], p=[.25, .25, .2, .15, .1, .05]))))
    return out
