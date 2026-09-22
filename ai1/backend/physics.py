"""Simple transparent drone-energy and wind model for an academic prototype."""
from math import sqrt

def vector_norm(x, y, z=0.0):
    return sqrt(x*x + y*y + z*z)

def effective_ground_speed(cruise_speed, vx, vy, vz, wind_sensitivity=0.35):
    wind = vector_norm(vx, vy, vz)
    # A bounded model: headwind raises time/energy; tailwind reduces it.
    return max(1.0, cruise_speed - wind_sensitivity * wind)

def route_metrics(path, drone, wind):
    if len(path) < 2:
        return {"distance_m": 0.0, "time_s": 0.0, "energy_wh": 0.0, "battery_remaining_wh": drone.battery_wh}

    distance = 0.0
    climb = 0.0
    for a, b in zip(path, path[1:]):
        dx = b["x"] - a["x"]
        dy = b["y"] - a["y"]
        dz = b["z"] - a["z"]
        d = vector_norm(dx, dy, dz)
        distance += d
        climb += max(0.0, dz)

    speed = effective_ground_speed(
        drone.cruise_speed_mps, wind.x, wind.y, wind.z, drone.wind_sensitivity
    )
    time_s = distance / speed
    # Baseline propulsion energy + climb penalty + wind penalty.
    wind_mag = vector_norm(wind.x, wind.y, wind.z)
    wind_factor = 1.0 + min(1.0, wind_mag / max(drone.cruise_speed_mps, 1.0)) * drone.wind_sensitivity
    energy = (drone.base_power_w * time_s / 3600.0) * wind_factor
    energy += climb * drone.climb_power_w_per_mps / 3600.0
    required = energy * drone.safety_margin
    return {
        "distance_m": round(distance, 2),
        "time_s": round(time_s, 2),
        "energy_wh": round(required, 2),
        "battery_remaining_wh": round(drone.battery_wh - required, 2),
        "wind_factor": round(wind_factor, 3),
        "battery_feasible": required <= drone.battery_wh,
    }
