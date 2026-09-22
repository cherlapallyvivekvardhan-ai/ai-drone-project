"""
drone_profile.py
----------------
Physical / performance characteristics of a delivery drone.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DroneProfile:
    name: str = "Falcon-1"
    max_speed_kmh: float = 45.0          # cruise speed
    max_payload_kg: float = 2.5
    cruise_altitude_m: float = 80.0
    power_consumption_w: float = 220.0   # average draw while cruising, no payload
    payload_power_factor: float = 35.0   # extra watts per kg of payload carried

    def power_draw_w(self, payload_kg: float) -> float:
        """Instantaneous power draw (W) while cruising with the given payload."""
        return self.power_consumption_w + payload_kg * self.payload_power_factor

    def travel_time_hours(self, distance_km: float) -> float:
        if self.max_speed_kmh <= 0:
            raise ValueError("max_speed_kmh must be > 0")
        return distance_km / self.max_speed_kmh

    @classmethod
    def from_dict(cls, data: dict) -> "DroneProfile":
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})
