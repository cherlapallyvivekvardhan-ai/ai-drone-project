"""
battery_profile.py
-------------------
Battery pack characteristics + simple energy bookkeeping helpers.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class BatteryProfile:
    name: str = "LiPo-6S-16000"
    capacity_wh: float = 180.0
    reserve_pct: float = 20.0            # % capacity that must never be touched (safety reserve)
    charge_pct: float = 100.0            # current state of charge at the start of a mission

    @property
    def usable_wh(self) -> float:
        """Energy actually available for the mission (excludes the safety reserve)."""
        usable_pct = max(self.charge_pct - self.reserve_pct, 0.0)
        return self.capacity_wh * (usable_pct / 100.0)

    def pct_used_for(self, energy_wh: float) -> float:
        if self.capacity_wh <= 0:
            raise ValueError("capacity_wh must be > 0")
        return (energy_wh / self.capacity_wh) * 100.0

    def can_complete(self, energy_wh_required: float) -> bool:
        return energy_wh_required <= self.usable_wh

    @classmethod
    def from_dict(cls, data: dict) -> "BatteryProfile":
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})
