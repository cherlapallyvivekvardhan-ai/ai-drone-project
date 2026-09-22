"""
simulator.py
------------
Steps a drone along a Route, minute-by-minute, tracking:
  - cumulative distance / time
  - energy consumption (based on DroneProfile power draw)
  - battery state of charge (based on BatteryProfile)

Produces a SimulationResult with a full event timeline and any
warnings (low battery, no-fly zone crossings, mission abort).
"""

from __future__ import annotations

from entities.battery_profile import BatteryProfile
from entities.drone_profile import DroneProfile
from entities.route import Route
from entities.simulation import SimulationEvent, SimulationResult

LOW_BATTERY_WARNING_PCT = 15.0
STEP_MINUTES = 1.0


class DroneSimulator:
    def __init__(self, drone: DroneProfile, battery: BatteryProfile, payload_kg: float = 0.0):
        self.drone = drone
        self.battery = battery
        self.payload_kg = payload_kg

    def run(self, route: Route) -> SimulationResult:
        total_time_hours = self.drone.travel_time_hours(route.distance_km)
        total_time_min = total_time_hours * 60.0

        power_w = self.drone.power_draw_w(self.payload_kg)
        energy_used_wh = power_w * total_time_hours
        battery_pct_used = self.battery.pct_used_for(energy_used_wh)
        battery_pct_remaining = self.battery.charge_pct - battery_pct_used

        warnings: list[str] = []
        events: list[SimulationEvent] = []

        success = self.battery.can_complete(energy_used_wh)
        if not success:
            warnings.append(
                f"Insufficient battery: mission needs {battery_pct_used:.1f}% but only "
                f"{self.battery.usable_wh / self.battery.capacity_wh * 100:.1f}% usable capacity available."
            )

        if route.no_fly_zones_crossed:
            warnings.append(
                "Route crosses restricted airspace: " + ", ".join(route.no_fly_zones_crossed)
            )

        # Build a simple timeline sampled every STEP_MINUTES.
        steps = max(int(total_time_min // STEP_MINUTES), 1)
        for i in range(steps + 1):
            t = min(i * STEP_MINUTES, total_time_min)
            fraction = 0.0 if total_time_min == 0 else t / total_time_min
            dist_so_far = route.distance_km * fraction
            batt_so_far = self.battery.charge_pct - battery_pct_used * fraction

            message = ""
            if batt_so_far <= LOW_BATTERY_WARNING_PCT and not any("low battery" in w for w in warnings):
                message = "low battery threshold reached"
                warnings.append(f"Battery dropped below {LOW_BATTERY_WARNING_PCT:.0f}% at t={t:.1f} min")

            events.append(SimulationEvent(
                time_min=round(t, 2),
                distance_km=round(dist_so_far, 3),
                battery_pct_remaining=round(batt_so_far, 2),
                message=message,
            ))

        return SimulationResult(
            success=success,
            total_time_min=total_time_min,
            total_distance_km=route.distance_km,
            energy_used_wh=energy_used_wh,
            battery_pct_used=battery_pct_used,
            battery_pct_remaining=battery_pct_remaining,
            events=events,
            warnings=warnings,
        )
