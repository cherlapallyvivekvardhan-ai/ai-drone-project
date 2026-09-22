from pathlib import Path
import json
from backend.models import MissionRequest
from ai_engine.astar import astar
from ai_engine.dijkstra import dijkstra
from ai_engine.path_planner import rasterize_zones, normalize_grid_point, smooth_path, grid_to_world
from ai_engine.route_optimizer import choose_route
from backend.physics import route_metrics

def load_zones_file(obj):
    """Accept native zones JSON or GeoJSON FeatureCollection."""
    if isinstance(obj, dict) and "zones" in obj:
        return obj["zones"]
    if isinstance(obj, dict) and obj.get("type") == "FeatureCollection":
        zones = []
        for idx, feature in enumerate(obj.get("features", [])):
            props = feature.get("properties") or {}
            geom = feature.get("geometry") or {}
            coords = geom.get("coordinates", [])
            flat = []
            def walk(v):
                if isinstance(v, (list, tuple)):
                    if len(v) >= 2 and all(isinstance(n, (int,float)) for n in v[:2]):
                        flat.append(v)
                    else:
                        for child in v:
                            walk(child)
            walk(coords)
            if not flat:
                continue
            xs = [p[0] for p in flat]; ys = [p[1] for p in flat]
            # GeoJSON x/y are treated as the platform's local coordinate units.
            zones.append({
                "id": props.get("id", f"geojson-{idx+1}"),
                "name": props.get("name", f"GeoJSON zone {idx+1}"),
                "type": props.get("type", "obstacle"),
                "min_x": min(xs), "max_x": max(xs),
                "min_y": min(ys), "max_y": max(ys),
                "min_z": props.get("min_z", 0),
                "max_z": props.get("max_z", 1000),
            })
        return zones
    raise ValueError("JSON must contain 'zones' or be a GeoJSON FeatureCollection.")

def build_mission(req: MissionRequest):
    grid = req.grid
    zones = [z.model_dump() for z in req.zones]
    margin = grid.cell_size_m * 0.25
    occupied = rasterize_zones(zones, grid, margin)

    start = normalize_grid_point(req.start.model_dump(), grid)
    goal = normalize_grid_point(req.destination.model_dump(), grid)

    if start in occupied:
        raise ValueError("Start point is inside/too close to a restricted zone.")
    if goal in occupied:
        raise ValueError("Destination point is inside/too close to a restricted zone.")

    candidates = []
    path_a, expanded_a = astar(occupied, start, goal, (grid.size_x, grid.size_y, grid.size_z))
    if path_a:
        smooth_a = smooth_path(path_a, occupied, grid)
        m_a = route_metrics(smooth_a, req.drone, req.wind)
        candidates.append({"algorithm":"A*", "path":smooth_a, "metrics":m_a, "expanded_nodes":expanded_a})

    if req.algorithm.lower() in ("dijkstra", "compare", "astar"):
        path_d, expanded_d = dijkstra(occupied, start, goal, (grid.size_x, grid.size_y, grid.size_z))
        if path_d:
            smooth_d = smooth_path(path_d, occupied, grid)
            m_d = route_metrics(smooth_d, req.drone, req.wind)
            candidates.append({"algorithm":"Dijkstra", "path":smooth_d, "metrics":m_d, "expanded_nodes":expanded_d})

    if not candidates:
        raise ValueError("No collision-free route exists in the configured 3D grid.")

    selected = choose_route(candidates) if req.optimize else candidates[0]
    return {
        "status": "success",
        "selected_algorithm": selected["algorithm"],
        "selected_route": selected["path"],
        "selected_metrics": selected["metrics"],
        "expanded_nodes": selected["expanded_nodes"],
        "grid": grid.model_dump(),
        "start": req.start.model_dump(),
        "destination": req.destination.model_dump(),
        "zones": zones,
        "candidates": [
            {
                "algorithm": c["algorithm"],
                "metrics": c["metrics"],
                "expanded_nodes": c["expanded_nodes"],
                "waypoints": len(c["path"])
            } for c in candidates
        ]
    }
