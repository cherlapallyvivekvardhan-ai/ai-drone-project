"""
user.py
-------
Minimal operator/user record -- kept simple since this is a
single-user, local, offline planning tool (no auth/network layer).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class User:
    username: str = "operator"
    role: str = "pilot"  # pilot | admin | viewer

    def __str__(self) -> str:
        return f"{self.username} ({self.role})"
