"""Obstacle rasterization, grid conversion, collision checks and smoothing."""
from math import sqrt

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def world_to_grid(p, grid):
    return (
        int(round(p["x"] / grid.cell_size_m)),
        int(round(p["y"] / grid.cell_size_m)),
        int(round(p["z"] / grid.cell_size_m)),
    )

def grid_to_world(p, grid):
    return {
        "x": round(p[0] * grid.cell_size_m, 2),
        "y": round(p[1] * grid.cell_size_m, 2),
        "z": round(p[2] * grid.cell_size_m, 2),
    }

def normalize_grid_point(p, grid):
    gx, gy, gz = world_to_grid(p, grid)
    return (
        clamp(gx, 0, grid.size_x-1),
        clamp(gy, 0, grid.size_y-1),
        clamp(gz, 0, grid.size_z-1),
    )

def point_in_zone(x,y,z,zone,margin=0.0):
    return (
        zone["min_x"]-margin <= x <= zone["max_x"]+margin and
        zone["min_y"]-margin <= y <= zone["max_y"]+margin and
        zone["min_z"]-margin <= z <= zone["max_z"]+margin
    )

def rasterize_zones(zones, grid, margin_m=0.0):
    occ = set()
    for ix in range(grid.size_x):
        x = ix * grid.cell_size_m
        for iy in range(grid.size_y):
            y = iy * grid.cell_size_m
            for iz in range(grid.size_z):
                z = iz * grid.cell_size_m
                if any(point_in_zone(x,y,z,zone,margin_m) for zone in zones):
                    occ.add((ix,iy,iz))
    return occ

def line_clear(a,b,occupied,grid):
    steps = max(2, int(
        sqrt((b["x"]-a["x"])**2 + (b["y"]-a["y"])**2 + (b["z"]-a["z"])**2)
        / max(grid.cell_size_m/2, 0.1)
    ))
    for i in range(steps+1):
        t = i/steps
        p = {
            "x": a["x"] + (b["x"]-a["x"])*t,
            "y": a["y"] + (b["y"]-a["y"])*t,
            "z": a["z"] + (b["z"]-a["z"])*t,
        }
        gp = normalize_grid_point(p, grid)
        if gp in occupied:
            return False
    return True

def smooth_path(path, occupied, grid):
    if len(path) <= 2:
        return path
    world = [grid_to_world(p, grid) for p in path]
    out = [world[0]]
    i = 0
    while i < len(world)-1:
        j = len(world)-1
        while j > i+1 and not line_clear(world[i], world[j], occupied, grid):
            j -= 1
        out.append(world[j])
        i = j
    return out
