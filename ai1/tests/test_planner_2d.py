from ai_engine.path_planner import plan_route


def test_astar_route():

    start = {
        "x": 2,
        "y": 2
    }

    destination = {
        "x": 10,
        "y": 10
    }

    obstacles = [
        {
            "x": 5,
            "y": 5
        }
    ]

    route = plan_route(
        start,
        destination,
        obstacles,
        "astar"
    )

    assert route[0] == start
    assert route[-1] == destination


def test_dijkstra_route():

    start = {
        "x": 1,
        "y": 1
    }

    destination = {
        "x": 8,
        "y": 8
    }

    route = plan_route(
        start,
        destination,
        [],
        "dijkstra"
    )

    assert route[0] == start
    assert route[-1] == destination
