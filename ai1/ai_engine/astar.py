import heapq
import math


def heuristic(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2
    )


def get_neighbors(node, width, height):
    x, y = node

    directions = [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1)
    ]

    neighbors = []

    for dx, dy in directions:
        nx = x + dx
        ny = y + dy

        if 0 <= nx < width and 0 <= ny < height:
            neighbors.append((nx, ny))

    return neighbors


def astar(start, goal, obstacles, width=50, height=50):

    obstacle_set = {
        (int(o["x"]), int(o["y"]))
        for o in obstacles
    }

    start = (int(start["x"]), int(start["y"]))
    goal = (int(goal["x"]), int(goal["y"]))

    if start in obstacle_set:
        raise ValueError("Start point is inside an obstacle.")

    if goal in obstacle_set:
        raise ValueError("Destination is inside an obstacle.")

    open_set = []

    heapq.heappush(
        open_set,
        (0, start)
    )

    came_from = {}

    g_score = {
        start: 0
    }

    while open_set:

        _, current = heapq.heappop(open_set)

        if current == goal:

            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()

            return [
                {
                    "x": x,
                    "y": y
                }
                for x, y in path
            ]

        for neighbor in get_neighbors(
            current,
            width,
            height
        ):

            if neighbor in obstacle_set:
                continue

            tentative_g = (
                g_score[current]
                + heuristic(current, neighbor)
            )

            if tentative_g < g_score.get(
                neighbor,
                float("inf")
            ):

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g

                f_score = (
                    tentative_g
                    + heuristic(neighbor, goal)
                )

                heapq.heappush(
                    open_set,
                    (f_score, neighbor)
                )

    raise ValueError(
        "No valid route could be found."
    )
