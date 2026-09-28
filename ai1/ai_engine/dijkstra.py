import heapq


def dijkstra(start, goal, obstacles, width=50, height=50):

    obstacle_set = {
        (int(o["x"]), int(o["y"]))
        for o in obstacles
    }

    start = (int(start["x"]), int(start["y"]))
    goal = (int(goal["x"]), int(goal["y"]))

    queue = [(0, start)]
    distances = {start: 0}
    previous = {}

    while queue:

        current_distance, current = heapq.heappop(queue)

        if current == goal:

            path = [current]

            while current in previous:
                current = previous[current]
                path.append(current)

            path.reverse()

            return [
                {"x": x, "y": y}
                for x, y in path
            ]

        x, y = current

        neighbors = [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1)
        ]

        for neighbor in neighbors:

            nx, ny = neighbor

            if not (
                0 <= nx < width
                and 0 <= ny < height
            ):
                continue

            if neighbor in obstacle_set:
                continue

            new_distance = current_distance + 1

            if new_distance < distances.get(
                neighbor,
                float("inf")
            ):

                distances[neighbor] = new_distance
                previous[neighbor] = current

                heapq.heappush(
                    queue,
                    (new_distance, neighbor)
                )

    raise ValueError("No route found.")
