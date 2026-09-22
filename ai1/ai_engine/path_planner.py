from ai_engine.astar import astar
from ai_engine.dijkstra import dijkstra
from ai_engine.route_optimizer import optimize_route


def plan_route(
    start,
    destination,
    obstacles,
    algorithm="astar"
):

    if algorithm.lower() == "astar":

        route = astar(
            start,
            destination,
            obstacles
        )

    elif algorithm.lower() == "dijkstra":

        route = dijkstra(
            start,
            destination,
            obstacles
        )

    else:
        raise ValueError(
            "Unsupported algorithm. "
            "Use astar or dijkstra."
        )

    route = optimize_route(route)

    return route
