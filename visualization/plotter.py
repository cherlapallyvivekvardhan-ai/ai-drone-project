"""
plotter.py
----------
Renders the grid, static obstacles, no-fly zones and the planned
route to a PNG image using matplotlib.

matplotlib is an OPTIONAL dependency: if it isn't installed, plot()
prints a friendly message and returns None instead of raising, so the
core CLI always works even in a bare-bones Python environment.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from entities.no_fly_zone import NoFlyZone

Point = Tuple[int, int]


def plot_route(
    width: int,
    height: int,
    blocked_cells: set,
    no_fly_zones: List[NoFlyZone],
    route_path: List[Point],
    stops: List[Point],
    output_path: str = "route_map.png",
) -> Optional[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")  # headless-safe backend
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except ImportError:
        print("[plotter] matplotlib not installed -- skipping image export. "
              "Run 'pip install matplotlib' to enable route visualization.")
        return None

    fig, ax = plt.subplots(figsize=(9, 9))

    # Static obstacles
    if blocked_cells:
        bx = [c[0] for c in blocked_cells]
        by = [c[1] for c in blocked_cells]
        ax.scatter(bx, by, c="#444444", marker="s", s=18, label="Obstacle")

    # No-fly zones
    for zone in no_fly_zones:
        circle = Circle(zone.center, zone.radius_cells, color="#e74c3c", alpha=0.25)
        ax.add_patch(circle)
        ax.annotate(zone.name, zone.center, color="#c0392b", fontsize=8, ha="center")

    # Planned path
    if route_path:
        xs = [p[0] for p in route_path]
        ys = [p[1] for p in route_path]
        ax.plot(xs, ys, c="#2980b9", linewidth=2, label="Flight path")

    # Stops (depot + deliveries)
    if stops:
        sx = [p[0] for p in stops]
        sy = [p[1] for p in stops]
        ax.scatter(sx, sy, c="#27ae60", marker="o", s=90, zorder=5, label="Stop")
        ax.annotate("Depot", stops[0], color="#145a32", fontsize=9, fontweight="bold")
        for i, p in enumerate(stops[1:], start=1):
            ax.annotate(f"D{i}", p, color="#145a32", fontsize=9)

    ax.set_xlim(-1, width)
    ax.set_ylim(-1, height)
    ax.set_aspect("equal")
    ax.set_title("Drone Delivery Route")
    ax.legend(loc="upper right", fontsize=8)
    ax.invert_yaxis()

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path
