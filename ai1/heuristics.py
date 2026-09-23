"""
heuristics.py
Multi-factor cost calculations and heuristic estimation functions.
"""

import numpy as np
from typing import Tuple
from environment import AirspaceEnvironment


class DronePhysicsModel:
    """Calculates physical energy consumption based on payload and wind force."""

    def __init__(self, base_weight: float = 3.0, payload_weight: float = 1.5, base_power_draw: float = 10.0):
        self.total_mass = base_weight + payload_weight  # Total drone mass in kg
        self.base_power = base_power_draw               # Base power consumption rate (Watts)

    def calculate_step_energy(self, current: Tuple[int, int, int], neighbor: Tuple[int, int, int], wind_vec: np.ndarray) -> float:
        """
        Calculates energy cost (Joules/Units) to transition between two adjacent 3D points.
        Accounts for headwind resistance and vertical ascent work.
        """
        move_vector = np.array(neighbor) - np.array(current)
        distance = np.linalg.norm(move_vector)
        if distance == 0:
            return 0.0

        # Movement direction unit vector
        direction = move_vector / distance

        # Relative velocity facing wind: opposing wind increases drag energy
        relative_wind = np.dot(direction, -wind_vec)
        headwind_penalty = max(0.0, relative_wind) * 1.5  # Drag factor

        # Vertical movement penalty (gravity exertion during climbing)
        climb_penalty = max(0.0, move_vector[2]) * 2.0 * self.total_mass

        # Total dynamic power factor
        effective_cost = distance * (self.total_mass + headwind_penalty) + climb_penalty
        return effective_cost


class MultiFactorHeuristic:
    """Informed Heuristic balancing Distance, Energy, and Risk factors."""

    def __init__(self, w_dist: float = 1.0, w_energy: float = 1.5, w_risk: float = 2.5):
        self.w_dist = w_dist
        self.w_energy = w_energy
        self.w_risk = w_risk

    def compute(self, current: Tuple[int, int, int], goal: Tuple[int, int, int],
                env: AirspaceEnvironment, physics: DronePhysicsModel) -> float:
        """
        Calculates total h(n) = w_d * Distance + w_e * Estimated Energy + w_r * Risk Penalty
        """
        curr_np = np.array(current)
        goal_np = np.array(goal)

        # 1. Distance Metric (Euclidean Distance in 3D)
        h_dist = np.linalg.norm(goal_np - curr_np)

        # 2. Estimated Energy Consumption to Goal against global wind
        estimated_step_energy = physics.calculate_step_energy(current, goal, env.wind_vector)
        h_energy = estimated_step_energy

        # 3. Risk Factor: Proximity to static obstacles and no-fly zones
        h_risk = 0.0
        search_radius = 2
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                for dz in range(-search_radius, search_radius + 1):
                    check_pos = (current[0] + dx, current[1] + dy, current[2] + dz)
                    if env.in_bounds(check_pos) and env.is_obstacle(check_pos):
                        dist = np.linalg.norm([dx, dy, dz])
                        if dist > 0:
                            h_risk += 1.0 / dist

        return (self.w_dist * h_dist) + (self.w_energy * h_energy) + (self.w_risk * h_risk)
