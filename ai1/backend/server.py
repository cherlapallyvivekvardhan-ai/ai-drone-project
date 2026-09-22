from flask import Flask, request, jsonify, send_from_directory
from ai_engine.path_planner import plan_route
from backend.physics import calculate_route_metrics
import json
import os

app = Flask(__name__, static_folder="../frontend")


@app.route("/")
def home():
    return send_from_directory("../frontend", "index.html")


@app.route("/api/default-zones", methods=["GET"])
def default_zones():
    path = os.path.join(
        os.path.dirname(__file__),
        "../data/default_zones.json"
    )

    with open(path, "r") as file:
        data = json.load(file)

    return jsonify(data)


@app.route("/api/plan-route", methods=["POST"])
def plan_drone_route():

    data = request.get_json()

    start = data.get("start")
    destination = data.get("destination")
    obstacles = data.get("obstacles", [])
    algorithm = data.get("algorithm", "astar")
    drone_speed = float(data.get("drone_speed", 10))

    if not start or not destination:
        return jsonify({
            "success": False,
            "error": "Start and destination are required."
        }), 400

    try:
        route = plan_route(
            start=start,
            destination=destination,
            obstacles=obstacles,
            algorithm=algorithm
        )

        metrics = calculate_route_metrics(
            route,
            drone_speed
        )

        return jsonify({
            "success": True,
            "algorithm": algorithm,
            "route": route,
            "distance": metrics["distance"],
            "flight_time": metrics["flight_time"],
            "waypoints": len(route)
        })

    except Exception as error:
        return jsonify({
            "success": False,
            "error": str(error)
        }), 500


if __name__ == "__main__":
    app.run(
        host="127.0.0.1",
        port=5000,
        debug=True
    )
