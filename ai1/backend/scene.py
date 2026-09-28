"""Milestones 1, 4 and 5 - pydeck scene: 3D city, red no-fly cylinders, neon ribbon, drone, cameras."""
from __future__ import annotations
import math
import numpy as np
import pydeck as pdk
from backend.geo import to_lnglat
from backend.physics import sample

NEON, CORE, ORANGE = [0, 255, 225, 70], [125, 255, 240, 255], [255, 159, 28, 255]


def _ll(origin, xyz):
    lng, lat = to_lnglat(origin, xyz[:, 0], xyz[:, 1])
    return np.column_stack([lng, lat] + ([xyz[:, 2]] if xyz.shape[1] > 2 else []))


def circle(z, n=48):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.column_stack([z.x + z.r * np.cos(a), z.y + z.r * np.sin(a)])


class Scene:
    """Everything that does not change between animation frames is built once."""

    def __init__(self, origin, world, path, zones, buildings):
        self.o, self.w, self.path, self.zones = origin, world, path, zones
        self.frames = sample(path, 6)                                         # drone positions / headings
        self.lnglat_path = _ll(origin, path).round(6).tolist()
        dense = _ll(origin, self.frames[:, :3]).round(6).tolist()             # smooth 3D polyline for the ribbon
        self.ribbon = [{"path": dense}]
        self.buildings = [{"polygon": _ll(origin, np.asarray(r)).round(6).tolist(), "h": h,
                           "color": [55 + min(h, 100) // 2, 70, 105 + min(h, 100) // 2, 235]}
                          for r, h in buildings if len(r) > 2]
        self.zone_polys = [{"polygon": _ll(origin, circle(z)).round(6).tolist(), "name": f"Restricted Airspace Zone {z.name}"}
                           for z in zones]
        self.pads = [{"pos": _ll(origin, np.array([[*p, 0.0]]))[0].tolist(), "c": c}
                     for p, c in ((path[0][:2], [46, 204, 113]), (path[-1][:2], [231, 76, 60]))]
        bx = [world.minx, world.maxx]; by = [world.miny, world.maxy]
        self.bbox = [{"polygon": _ll(origin, np.array([[bx[0], by[0]], [bx[1], by[0]], [bx[1], by[1]], [bx[0], by[1]]])).tolist()}]
        self.center = _ll(origin, np.array([[(bx[0] + bx[1]) / 2, (by[0] + by[1]) / 2]]))[0].tolist()
        ext = max(bx[1] - bx[0], by[1] - by[0])
        self.zoom = float(math.log2(156543 * math.cos(math.radians(self.center[1])) * 900 / (ext * 1.15)))

    def drone(self, progress):
        f = self.frames[min(len(self.frames) - 1, int(round(progress * (len(self.frames) - 1))))]
        return f, _ll(self.o, f[None, :3])[0].tolist()

    # ---- Milestone 5: cameras -------------------------------------------------------------
    def view(self, mode, progress, bearing, pitch):
        if mode == "FPV":                                                    # chase camera behind the drone
            f, p = self.drone(progress)
            ahead = _ll(self.o, np.array([[f[0] + f[3] * 50, f[1] + f[4] * 50]]))[0].tolist()
            return pdk.ViewState(latitude=ahead[1], longitude=ahead[0], zoom=17.6, pitch=78, bearing=float(math.degrees(math.atan2(f[3], f[4])) % 360))
        return pdk.ViewState(latitude=self.center[1], longitude=self.center[0], zoom=self.zoom, pitch=pitch, bearing=bearing)

    def _drone_layers(self, progress):
        f, p = self.drone(progress)
        d = [{"pos": [p[0], p[1], float(f[2]) + 4.0]}]
        return [pdk.Layer("ScatterplotLayer", d, get_position="pos", get_radius=26, radius_units="meters", get_fill_color=[255, 159, 28, 90]),
                pdk.Layer("ScatterplotLayer", d, get_position="pos", get_radius=10, radius_units="meters", get_fill_color=ORANGE)]

    def deck3d(self, progress=0.0, mode="Orbit", bearing=-20, pitch=60):
        L = [pdk.Layer("PolygonLayer", self.buildings, get_polygon="polygon", extruded=True, get_elevation="h",
                       get_fill_color="color", material=True),
             pdk.Layer("PolygonLayer", self.zone_polys, get_polygon="polygon", extruded=True, get_elevation=300,
                       get_fill_color=[255, 30, 30, 80], pickable=True, auto_highlight=True),
             pdk.Layer("PathLayer", self.ribbon, get_path="path", get_width=16, width_units="meters", get_color=NEON, billboard=True),
             pdk.Layer("PathLayer", self.ribbon, get_path="path", get_width=4, width_units="meters", get_color=CORE, billboard=True),
             pdk.Layer("ScatterplotLayer", self.pads, get_position="pos", get_radius=14, radius_units="meters", get_fill_color="c")]
        return pdk.Deck(layers=L + self._drone_layers(progress), initial_view_state=self.view(mode, progress, bearing, pitch),
                        map_style="dark", tooltip={"text": "{name}"})

    def deck2d(self, progress=0.0):
        L = [pdk.Layer("PolygonLayer", self.bbox, get_polygon="polygon", filled=False, stroked=True, get_line_color=[255, 255, 255, 200], get_line_width=3, line_width_units="pixels"),
             pdk.Layer("PolygonLayer", self.zone_polys, get_polygon="polygon", get_fill_color=[255, 30, 30, 110]),
             pdk.Layer("PathLayer", [{"path": [p[:2] for p in self.lnglat_path]}], get_path="path", get_width=3, width_units="pixels", get_color=[0, 255, 225, 255]),
             pdk.Layer("ScatterplotLayer", self.pads, get_position="pos", get_radius=7, radius_units="pixels", get_fill_color="c")]
        f, p = self.drone(progress)
        L.append(pdk.Layer("ScatterplotLayer", [{"pos": [p[0], p[1]]}], get_position="pos", get_radius=7, radius_units="pixels", get_fill_color=ORANGE))
        return pdk.Deck(layers=L, initial_view_state=pdk.ViewState(latitude=self.center[1], longitude=self.center[0], zoom=self.zoom - .2, pitch=0),
                        map_style="dark", tooltip={"text": "{name}"})
