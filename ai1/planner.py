"""
planner.py
Global Macro-Routing via 3D A* Search and Micro-Adjustment Layer for Dynamic Obstacle Avoidance.
"""

import heapq
import numpy as np
from typing import List, Tuple, Dict, Optional, Set
from environment import AirspaceEnvironment
from heuristics import MultiFactorHeuristic, DronePhysicsModel


class AStar3DPlanner:
    """3D A* Search Path Planner with multi-factor heuristic."""

    def __init__(self, env: AirspaceEnvironment, physics: DronePhysicsModel, heuristic: MultiFactorHeuristic):
        self.env = env
        self.physics = physics
        self.heuristic = heuristic

    def get_neighbors(self, pos: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        """Generates 26-connected 3D adjacent neighbors."""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neighbor = (pos[0] + dx, pos[1] + dy, pos[2] + dz)
                    if self.env.in_bounds(neighbor) and not self.env.is_obstacle(neighbor):
                        neighbors.append(neighbor)
        return neighbors

    def plan_path(self, start: Tuple[int, int, int], goal: Tuple[int, int, int]) -> Optional[List[Tuple[int, int, int]]]:
        """Runs 3D A* Search algorithm to compute optimal macro path."""
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
        g_score: Dict[Tuple[int, int, int], float] = {start: 0.0}
        f_score: Dict[Tuple[int, int, int], float] = {
            start: self.heuristic.compute(start, goal, self.env, self.physics)
        }

        open_set_hash: Set[Tuple[int, int, int]] = {start}

        while open_set:
            _, current = heapq.heappop(open_set)
            open_set_hash.remove(current)

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for neighbor in self.get_neighbors(current):
                step_cost = self.physics.calculate_step_energy(current, neighbor, self.env.wind_vector)
                tentative_g_score = g_score[current] + step_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f = tentative_g_score + self.heuristic.compute(neighbor, goal, self.env, self.physics)
                    f_score[neighbor] = f

                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f, neighbor))
                        open_set_hash.add(neighbor)

        return None  # Path not found


class ReactiveMicroAdjuster:
    """Artificial Potential Field (APF) local controller for real-time dynamic obstacle avoidance."""

    def __init__(self, env: AirspaceEnvironment, k_att: float = 1.0, k_rep: float = 5.0, influence_dist: float = 3.0):
        self.env = env
        self.k_att = k_att  # Attraction gain toward sub-goal
        self.k_rep = k_rep  # Repulsion gain away from dynamic obstacle
        self.influence_dist = influence_dist

    def compute_next_step(self, current_pos: Tuple[int, int, int], target_waypoint: Tuple[int, int, int]) -> Tuple[int, int, int]:
        """Computes micro-adjustment step if dynamic obstacle is blocking immediate path."""
        pos_np = np.array(current_pos, dtype=float)
        target_np = np.array(target_waypoint, dtype=float)

        # 1. Attractive force towards planned waypoint
        f_att = self.k_att * (target_np - pos_np)

        # 2. Repulsive force from dynamic obstacles
        f_rep = np.zeros(3)
        for obs in self.env.dynamic_obstacles:
            diff = pos_np - obs.position
            dist = np.linalg.norm(diff)
            if dist < self.influence_dist:
                rep_magnitude = self.k_rep * ((1.0 / max(dist, 0.1)) - (1.0 / self.influence_dist)) / (dist ** 2)
                f_rep += (diff / max(dist, 0.1)) * rep_magnitude

        total_force = f_att + f_rep

        # Determine best discrete integer grid step (+1, 0, -1) along result vector
        step = np.sign(total_force).astype(int)
        next_candidate = (current_pos[0] + step[0], current_pos[1] + step[1], current_pos[2] + step[2])

        # Validate movement
        if self.env.in_bounds(next_candidate) and not self.env.is_obstacle(next_candidate):
            return next_candidate

        # Fallback to current position if local path is fully blocked
        return current_pos
