"""
app.py
Streamlit front-end for the AI Delivery Drone Path Planning simulation.

Run with:
    streamlit run app.py
"""

import time
import numpy as np
import streamlit as st
import plotly.graph_objects as go

from environment import AirspaceEnvironment, DynamicObstacle
from heuristics import MultiFactorHeuristic, DronePhysicsModel
from planner import AStar3DPlanner, ReactiveMicroAdjuster

st.set_page_config(page_title="Drone Path Planning", layout="wide")

st.title("AI Delivery Drone Path Planning Simulation")
st.caption("3D A* macro-routing + Artificial Potential Field micro-adjustment for dynamic obstacle avoidance")

# ---------------------------------------------------------------------------
# Sidebar controls
# ---------------------------------------------------------------------------
with st.sidebar:
    st.header("Simulation Parameters")

    st.subheader("Airspace")
    bx = st.slider("Bounds X", 10, 60, 30)
    by = st.slider("Bounds Y", 10, 60, 30)
    bz = st.slider("Bounds Z (altitude)", 5, 40, 20)

    st.subheader("Wind")
    wx = st.slider("Wind X", -5.0, 5.0, -2.0, 0.5)
    wy = st.slider("Wind Y", -5.0, 5.0, 1.5, 0.5)
    wz = st.slider("Wind Z", -5.0, 5.0, 0.0, 0.5)

    st.subheader("Drone Physics")
    base_weight = st.slider("Base weight (kg)", 0.5, 10.0, 3.5, 0.5)
    payload_weight = st.slider("Payload weight (kg)", 0.0, 10.0, 2.0, 0.5)

    st.subheader("Heuristic Weights")
    w_dist = st.slider("Distance weight", 0.0, 5.0, 1.0, 0.1)
    w_energy = st.slider("Energy weight", 0.0, 5.0, 1.8, 0.1)
    w_risk = st.slider("Risk weight", 0.0, 5.0, 3.0, 0.1)

    st.subheader("Start / Goal")
    start_x = st.number_input("Start X", 0, bx - 1, min(2, bx - 1))
    start_y = st.number_input("Start Y", 0, by - 1, min(2, by - 1))
    start_z = st.number_input("Start Z", 0, bz - 1, min(1, bz - 1))
    goal_x = st.number_input("Goal X", 0, bx - 1, min(27, bx - 1))
    goal_y = st.number_input("Goal Y", 0, by - 1, min(27, by - 1))
    goal_z = st.number_input("Goal Z", 0, bz - 1, min(5, bz - 1))

    run_button = st.button("Run Simulation", type="primary", use_container_width=True)


def build_environment():
    bounds = (bx, by, bz)
    wind_vector = (wx, wy, wz)
    env = AirspaceEnvironment(bounds, wind_vector=wind_vector)

    # Static buildings (clamped to fit whatever bounds the user picked)
    env.add_static_building(
        min_corner=(min(5, bx - 1), min(5, by - 1), 0),
        max_corner=(min(12, bx - 1), min(12, by - 1), min(12, bz - 1)),
    )
    env.add_static_building(
        min_corner=(min(15, bx - 1), min(10, by - 1), 0),
        max_corner=(min(22, bx - 1), min(18, by - 1), min(15, bz - 1)),
    )

    # No-fly zone
    env.add_no_fly_zone(
        center=(min(10, bx - 1), min(22, by - 1), 0),
        radius=4,
        height_range=(0, bz - 1),
    )

    # Dynamic obstacles
    env.dynamic_obstacles.append(DynamicObstacle(start_pos=(min(8, bx - 1), min(15, by - 1), min(8, bz - 1)), velocity=(0, 1, 0)))
    env.dynamic_obstacles.append(DynamicObstacle(start_pos=(min(20, bx - 1), min(5, by - 1), min(10, bz - 1)), velocity=(-1, 0, 0)))

    return env


def run_simulation(start, goal):
    env = build_environment()
    physics = DronePhysicsModel(base_weight=base_weight, payload_weight=payload_weight)
    heuristic = MultiFactorHeuristic(w_dist=w_dist, w_energy=w_energy, w_risk=w_risk)

    t0 = time.perf_counter()
    macro_planner = AStar3DPlanner(env, physics, heuristic)
    macro_path = macro_planner.plan_path(start, goal)
    macro_calc_time = (time.perf_counter() - t0) * 1000

    if not macro_path:
        return env, None, None, macro_calc_time, None, False

    micro_adjuster = ReactiveMicroAdjuster(env)
    actual_flight_path = [start]
    current_pos = start
    total_energy_consumed = 0.0
    successful_delivery = False

    t_sim_start = time.perf_counter()
    for target_waypoint in macro_path[1:]:
        env.update_dynamic_environment()
        risk = env.get_dynamic_repulsion(current_pos, influence_radius=2.5)

        if risk > 0.5:
            next_step = micro_adjuster.compute_next_step(current_pos, target_waypoint)
        else:
            next_step = target_waypoint

        step_energy = physics.calculate_step_energy(current_pos, next_step, env.wind_vector)
        total_energy_consumed += step_energy
        current_pos = next_step
        actual_flight_path.append(current_pos)

        if current_pos == goal:
            successful_delivery = True

    sim_execution_time = (time.perf_counter() - t_sim_start) * 1000
    if actual_flight_path[-1] == goal:
        successful_delivery = True

    total_distance = sum(
        np.linalg.norm(np.array(actual_flight_path[i]) - np.array(actual_flight_path[i - 1]))
        for i in range(1, len(actual_flight_path))
    )

    metrics = {
        "status": successful_delivery,
        "distance": total_distance,
        "energy": total_energy_consumed,
        "planner_ms": macro_calc_time,
        "sim_ms": sim_execution_time,
        "waypoints": len(macro_path),
    }

    return env, macro_path, actual_flight_path, macro_calc_time, metrics, True


def make_3d_figure(env, macro_path, actual_path, start, goal):
    fig = go.Figure()

    # Static obstacles (buildings)
    if env.static_obstacles:
        sx, sy, sz = zip(*env.static_obstacles)
        fig.add_trace(go.Scatter3d(
            x=sx, y=sy, z=sz, mode="markers",
            marker=dict(size=3, color="gray", opacity=0.3),
            name="Buildings",
        ))

    # No-fly zones
    if env.no_fly_zones:
        nx, ny, nz = zip(*env.no_fly_zones)
        fig.add_trace(go.Scatter3d(
            x=nx, y=ny, z=nz, mode="markers",
            marker=dict(size=3, color="orange", opacity=0.15),
            name="No-Fly Zone",
        ))

    # Planned macro path
    if macro_path:
        mx, my, mz = zip(*macro_path)
        fig.add_trace(go.Scatter3d(
            x=mx, y=my, z=mz, mode="lines",
            line=dict(color="blue", width=4, dash="dash"),
            name="Global A* Planned Path",
        ))

    # Actual flown path
    if actual_path:
        ax_, ay_, az_ = zip(*actual_path)
        fig.add_trace(go.Scatter3d(
            x=ax_, y=ay_, z=az_, mode="lines",
            line=dict(color="red", width=6),
            name="Actual Flight Path (Reactive)",
        ))

    # Dynamic obstacle final positions
    for i, obs in enumerate(env.dynamic_obstacles):
        fig.add_trace(go.Scatter3d(
            x=[obs.position[0]], y=[obs.position[1]], z=[obs.position[2]],
            mode="markers", marker=dict(size=6, color="purple"),
            name="Dynamic Obstacle" if i == 0 else None,
            showlegend=(i == 0),
        ))

    # Start / goal
    fig.add_trace(go.Scatter3d(
        x=[start[0]], y=[start[1]], z=[start[2]], mode="markers",
        marker=dict(size=8, color="green", symbol="circle"), name="Warehouse (Start)",
    ))
    fig.add_trace(go.Scatter3d(
        x=[goal[0]], y=[goal[1]], z=[goal[2]], mode="markers",
        marker=dict(size=9, color="gold", symbol="diamond"), name="Customer (Goal)",
    ))

    fig.update_layout(
        scene=dict(
            xaxis_title="X (meters)",
            yaxis_title="Y (meters)",
            zaxis_title="Altitude Z (meters)",
        ),
        legend=dict(x=0, y=1),
        margin=dict(l=0, r=0, t=30, b=0),
        height=700,
    )
    return fig


# ---------------------------------------------------------------------------
# Main panel
# ---------------------------------------------------------------------------
if run_button:
    start = (int(start_x), int(start_y), int(start_z))
    goal = (int(goal_x), int(goal_y), int(goal_z))

    with st.spinner("Computing global path and simulating flight..."):
        env, macro_path, actual_path, planner_time, metrics, ok = run_simulation(start, goal)

    if not ok:
        st.error("Global path planner failed to find a valid trajectory between start and goal.")
    else:
        c1, c2, c3, c4, c5 = st.columns(5)
        c1.metric("Delivery", "SUCCESS" if metrics["status"] else "FAILED")
        c2.metric("Flight Distance", f"{metrics['distance']:.2f} units")
        c3.metric("Energy Consumed", f"{metrics['energy']:.2f} J")
        c4.metric("Planner Time", f"{metrics['planner_ms']:.2f} ms")
        c5.metric("Waypoints", metrics["waypoints"])

        st.plotly_chart(make_3d_figure(env, macro_path, actual_path, start, goal), use_container_width=True)
else:
    st.info("Set parameters in the sidebar and click **Run Simulation** to compute a path.")
