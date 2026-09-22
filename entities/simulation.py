"""
simulation.py
-------------
Result of running a drone through a Route: timeline, battery usage,
and any warnings raised along the way (low battery, no-fly crossing).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class SimulationEvent:
    time_min: float
    distance_km: float
    battery_pct_remaining: float
    message: str


@dataclass
class SimulationResult:
    success: bool
    total_time_min: float
    total_distance_km: float
    energy_used_wh: float
    battery_pct_used: float
    battery_pct_remaining: float
    events: List[SimulationEvent] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def print_report(self) -> None:
        status = "✅ MISSION SUCCESSFUL" if self.success else "❌ MISSION ABORTED"
        print(f"\n{status}")
        print(f"  Total distance:      {self.total_distance_km:.2f} km")
        print(f"  Total flight time:   {self.total_time_min:.1f} min")
        print(f"  Energy used:         {self.energy_used_wh:.1f} Wh ({self.battery_pct_used:.1f}% of pack)")
        print(f"  Battery remaining:   {self.battery_pct_remaining:.1f}%")
        if self.warnings:
            print("  Warnings:")
            for w in self.warnings:
                print(f"    - {w}")
