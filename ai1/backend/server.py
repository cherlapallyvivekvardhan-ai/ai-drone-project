import json
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory

from ai_engine.path_planner import ALGORITHMS, plan_route_detailed
from backend.physics import calculate_route_metrics

ROOT = Path(__file__).resolve().parent.parent
FRONTEND = ROOT / "frontend"
ZONES = ROOT / "data" / "default_zones.json"

MAX_SIZE = 200
app = Flask(__name__, static_folder=None)


def error(message, status=400):
    return jsonify({"success": False, "error": message}), status


def parse_mission(data):
    """Validate the request body. Raises ValueError with a user-facing message."""
    if not isinstance(data, dict):
        raise ValueError("Request body must be JSON.")
    if not data.get("start") or not data.get("destination"):
        raise ValueError("Start and destination are required.")
    try:
        speed = float(data.get("drone_speed", 10))
        width = int(data.get("width", 50))
        height = int(data.get("height", 50))
    except (TypeError, ValueError):
        raise ValueError("Speed, width and height must be numbers.")
    if not (1 <= width <= MAX_SIZE and 1 <= height <= MAX_SIZE):
        raise ValueError(f"Map size must be between 1 and {MAX_SIZE}.")
    if speed <= 0:
        raise ValueError("Drone speed must be greater than zero.")
    obstacles = data.get("obstacles", [])
    if not isinstance(obstacles, list):
        raise ValueError("Obstacles must be a list of {x, y} points.")
    return {
        "start": data["start"], "destination": data["destination"],
        "obstacles": obstacles, "speed": speed,
        "width": width, "height": height,
        "smooth": bool(data.get("smooth", False)),
    }


def run_one(m, algorithm):
    result = plan_route_detailed(m["start"], m["destination"], m["obstacles"],
                                 algorithm, m["width"], m["height"], m["smooth"])
    metrics = calculate_route_metrics(result["route"], m["speed"])
    return {
        "algorithm": result["algorithm"],
        "route": result["route"],
        "distance": metrics["distance"],
        "distance_m": metrics["distance_m"],
        "flight_time": metrics["flight_time"],
        "waypoints": len(result["route"]),
        "expanded": result["expanded"],
        "explored": result["explored"],
        "compute_ms": result["compute_ms"],
    }


@app.route("/")
def home():
    return send_from_directory(FRONTEND, "index.html")


@app.route("/api/health")
def health():
    return jsonify({"status": "ok", "algorithms": sorted(ALGORITHMS)})


@app.route("/api/default-zones")
def default_zones():
    with open(ZONES, "r", encoding="utf-8") as f:
        return jsonify(json.load(f))


@app.route("/api/plan-route", methods=["POST"])
def plan_drone_route():
    try:
        m = parse_mission(request.get_json(silent=True))
        out = run_one(m, request.get_json(silent=True).get("algorithm", "astar"))
        return jsonify({"success": True, **out})
    except ValueError as e:
        # Bad input or "no route": the client can fix these, so not a 500.
        return error(str(e), 422 if "route" in str(e).lower() else 400)
    except Exception:
        app.logger.exception("Route planning failed")
        return error("Unexpected server error while planning the route.", 500)


@app.route("/api/compare", methods=["POST"])
def compare():
    try:
        m = parse_mission(request.get_json(silent=True))
        results = {name: run_one(m, name) for name in ("astar", "dijkstra")}
        return jsonify({"success": True, "results": results})
    except ValueError as e:
        return error(str(e), 422 if "route" in str(e).lower() else 400)
    except Exception:
        app.logger.exception("Comparison failed")
        return error("Unexpected server error while comparing algorithms.", 500)
