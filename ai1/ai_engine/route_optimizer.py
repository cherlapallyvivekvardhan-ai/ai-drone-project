"""Multi-objective route selection: distance, time, energy and battery feasibility."""
from .path_planner import line_clear

def score(metrics):
    # Lower is better. Infeasible battery routes get a strong penalty.
    penalty = 0 if metrics["battery_feasible"] else 100000
    return (
        metrics["distance_m"]
        + 2.0 * metrics["time_s"]
        + 8.0 * metrics["energy_wh"]
        + penalty
    )

def choose_route(candidates):
    valid = [c for c in candidates if c["path"]]
    if not valid:
        return None
    for c in valid:
        c["score"] = round(score(c["metrics"]), 3)
    return min(valid, key=lambda c: c["score"])
