def optimize_route(route):

    if len(route) <= 2:
        return route

    optimized = [route[0]]

    previous_direction = None

    for i in range(1, len(route)):

        if i == len(route) - 1:
            optimized.append(route[i])
            break

        current = route[i]
        previous = route[i - 1]
        next_point = route[i + 1]

        direction = (
            next_point["x"] - current["x"],
            next_point["y"] - current["y"]
        )

        if direction != previous_direction:
            optimized.append(current)

        previous_direction = direction

    return optimized
