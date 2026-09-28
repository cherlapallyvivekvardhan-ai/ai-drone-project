"""AI-based delivery drone path planning - one file, Python standard library only.

Run the app:    python drone_planner.py        (opens http://127.0.0.1:5000)
Run the tests:  python drone_planner.py test
"""
import heapq
import json
import math
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SIZE = 50          # map is SIZE x SIZE cells
CELL_METRES = 10   # one cell = 10 m
SQRT2 = math.sqrt(2)
MOVES = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

# Default map: your original 3x3 block plus a wall with gaps at both ends.
DEFAULT_OBSTACLES = sorted(
    {(x, y) for x in range(15, 18) for y in range(15, 18)}
    | {(30, y) for y in range(5, 40)}
)


# ---------------------------------------------------------------- planning
def find_path(start, goal, blocked, algorithm="astar"):
    """Shortest path on an 8-direction grid.

    Dijkstra is A* with no heuristic, so one function does both. Both find the
    same shortest distance; A* just searches fewer cells.
    Returns (path, cells_searched). Raises ValueError if no route exists.
    """
    def heuristic(cell):
        if algorithm != "astar":
            return 0
        dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return (dx + dy) + (SQRT2 - 2) * min(dx, dy)  # exact distance if map is empty

    queue = [(heuristic(start), 0.0, start)]  # (estimated total, cost so far, cell)
    cost = {start: 0.0}
    parent = {}
    done = set()

    while queue:
        _, g, cell = heapq.heappop(queue)
        if cell in done:
            continue
        done.add(cell)

        if cell == goal:
            path = [cell]
            while cell in parent:
                cell = parent[cell]
                path.append(cell)
            return path[::-1], len(done)

        x, y = cell
        for dx, dy in MOVES:
            nxt = (x + dx, y + dy)
            if not (0 <= nxt[0] < SIZE and 0 <= nxt[1] < SIZE) or nxt in blocked:
                continue
            # Don't squeeze diagonally between two touching obstacles.
            if dx and dy and ((x + dx, y) in blocked or (x, y + dy) in blocked):
                continue
            new_cost = g + (SQRT2 if dx and dy else 1.0)
            if new_cost < cost.get(nxt, math.inf):
                cost[nxt] = new_cost
                parent[nxt] = cell
                heapq.heappush(queue, (new_cost + heuristic(nxt), new_cost, nxt))

    raise ValueError("No route exists between start and destination.")


def keep_corners(path):
    """Drop points in the middle of straight stretches; keep only the turns."""
    if len(path) < 3:
        return path
    out = [path[0]]
    for prev, cur, nxt in zip(path, path[1:], path[2:]):
        if (cur[0] - prev[0], cur[1] - prev[1]) != (nxt[0] - cur[0], nxt[1] - cur[1]):
            out.append(cur)
    out.append(path[-1])
    return out


def plan(request):
    """Validate a request dict, plan the route, return a result dict."""
    def cell(name):
        try:
            c = (int(request[name]["x"]), int(request[name]["y"]))
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"'{name}' must look like {{\"x\": 1, \"y\": 2}}.")
        if not (0 <= c[0] < SIZE and 0 <= c[1] < SIZE):
            raise ValueError(f"The {name} {c} is outside the {SIZE}x{SIZE} map.")
        return c

    start, goal = cell("start"), cell("destination")
    algorithm = str(request.get("algorithm", "astar")).lower()
    if algorithm not in ("astar", "dijkstra"):
        raise ValueError("Algorithm must be 'astar' or 'dijkstra'.")
    try:
        speed = float(request.get("drone_speed", 10))
        blocked = {(int(o["x"]), int(o["y"])) for o in request.get("obstacles", [])}
    except (TypeError, ValueError, KeyError):
        raise ValueError("Speed must be a number and obstacles must be {x, y} points.")
    if speed <= 0:
        raise ValueError("Drone speed must be greater than zero.")
    for name, c in (("start", start), ("destination", goal)):
        if c in blocked:
            raise ValueError(f"The {name} {c} is inside an obstacle.")

    path, searched = find_path(start, goal, blocked, algorithm)
    route = keep_corners(path)
    cells = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
    metres = cells * CELL_METRES
    return {
        "success": True,
        "algorithm": algorithm,
        "route": [{"x": x, "y": y} for x, y in route],
        "distance_m": round(metres, 1),
        "flight_time": round(metres / speed, 1),
        "waypoints": len(route),
        "cells_searched": searched,
    }


# ------------------------------------------------------------- web server
PAGE = """<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Drone Path Planner</title>
<style>
body{margin:0;font:15px system-ui,sans-serif;background:#101827;color:#fff}
header{padding:16px 24px;background:#172238;border-bottom:1px solid #334155}
h1{margin:0;font-size:22px}
.layout{display:flex;flex-wrap:wrap;gap:24px;padding:24px}
.side{width:260px}
label{display:block;margin-top:12px}
input,select,button{width:100%;padding:9px;margin-top:4px;border:0;border-radius:6px;font:inherit}
input,select{background:#26344d;color:#fff}
button{background:#2563eb;color:#fff;font-weight:bold;cursor:pointer;margin-top:14px}
button:hover{background:#1d4ed8}
canvas{background:#0b1322;border:2px solid #334155;max-width:100%;cursor:pointer}
.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px;max-width:600px}
.stats div{background:#172238;padding:10px;border-radius:6px}
.stats small{color:#94a3b8;display:block}
#status{margin-top:12px;min-height:22px}
.err{color:#fca5a5}
</style></head><body>
<header><h1>Delivery Drone Path Planner</h1></header>
<div class="layout">
 <div class="side">
  <label>Start X <input id="sx" type="number" value="2" min="0" max="49"></label>
  <label>Start Y <input id="sy" type="number" value="2" min="0" max="49"></label>
  <label>Destination X <input id="dx" type="number" value="45" min="0" max="49"></label>
  <label>Destination Y <input id="dy" type="number" value="45" min="0" max="49"></label>
  <label>Drone speed (m/s) <input id="speed" type="number" value="10" min="1"></label>
  <label>Algorithm <select id="algo">
    <option value="astar">A* (fast)</option><option value="dijkstra">Dijkstra (baseline)</option>
  </select></label>
  <button onclick="plan()">Plan route</button>
  <button onclick="reset()">Reset obstacles</button>
  <p style="color:#94a3b8">Click the map to add or remove an obstacle. One cell is 10 m.</p>
 </div>
 <div>
  <canvas id="map" width="600" height="600"></canvas>
  <div class="stats">
   <div><small>Algorithm</small><b id="r_algo">-</b></div>
   <div><small>Distance</small><b id="r_dist">-</b></div>
   <div><small>Flight time</small><b id="r_time">-</b></div>
   <div><small>Searched</small><b id="r_seen">-</b></div>
  </div>
  <div id="status">Ready.</div>
 </div>
</div>
<script>
const N = 50, C = 12, cv = document.getElementById("map"), g = cv.getContext("2d");
let blocked = new Set(), route = null;
const val = id => Number(document.getElementById(id).value);
const say = (t, bad) => { const s = document.getElementById("status"); s.textContent = t; s.className = bad ? "err" : ""; };

function draw() {
  g.clearRect(0, 0, 600, 600);
  g.strokeStyle = "#1e293b"; g.beginPath();
  for (let i = 0; i <= N; i++) { g.moveTo(i*C, 0); g.lineTo(i*C, 600); g.moveTo(0, i*C); g.lineTo(600, i*C); }
  g.stroke();
  g.fillStyle = "#f59e0b";
  blocked.forEach(k => { const [x, y] = k.split(",").map(Number); g.fillRect(x*C, y*C, C, C); });
  if (route) {
    g.strokeStyle = "#38bdf8"; g.lineWidth = 4; g.beginPath();
    route.forEach((p, i) => i ? g.lineTo(p.x*C + C/2, p.y*C + C/2) : g.moveTo(p.x*C + C/2, p.y*C + C/2));
    g.stroke(); g.lineWidth = 1;
  }
  dot(val("sx"), val("sy"), "#22c55e"); dot(val("dx"), val("dy"), "#ef4444");
}
function dot(x, y, color) { g.fillStyle = color; g.beginPath(); g.arc(x*C + C/2, y*C + C/2, 7, 0, 7); g.fill(); }

async function plan() {
  say("Planning...");
  const body = {
    start: {x: val("sx"), y: val("sy")}, destination: {x: val("dx"), y: val("dy")},
    drone_speed: val("speed"), algorithm: document.getElementById("algo").value,
    obstacles: [...blocked].map(k => { const [x, y] = k.split(",").map(Number); return {x, y}; })
  };
  const res = await fetch("/api/plan", {method: "POST", body: JSON.stringify(body)});
  const r = await res.json();
  if (!r.success) { route = null; draw(); return say(r.error, true); }
  route = r.route;
  document.getElementById("r_algo").textContent = r.algorithm;
  document.getElementById("r_dist").textContent = r.distance_m + " m";
  document.getElementById("r_time").textContent = r.flight_time + " s";
  document.getElementById("r_seen").textContent = r.cells_searched + " cells";
  draw(); say("Route found with " + r.waypoints + " waypoints.");
}
async function reset() {
  blocked = new Set((await (await fetch("/api/zones")).json()).obstacles.map(o => o.x + "," + o.y));
  route = null; draw(); say("Default obstacles loaded.");
}
cv.onclick = e => {
  const r = cv.getBoundingClientRect();
  const k = Math.floor((e.clientX - r.left) / r.width * N) + "," + Math.floor((e.clientY - r.top) / r.height * N);
  blocked.has(k) ? blocked.delete(k) : blocked.add(k);
  route ? plan() : draw();
};
reset();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, body, content_type="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/":
            self.reply(200, PAGE.encode(), "text/html; charset=utf-8")
        elif self.path == "/api/zones":
            self.reply(200, {"width": SIZE, "height": SIZE,
                             "obstacles": [{"x": x, "y": y} for x, y in DEFAULT_OBSTACLES]})
        else:
            self.reply(404, {"success": False, "error": "Not found."})

    def do_POST(self):
        if self.path != "/api/plan":
            return self.reply(404, {"success": False, "error": "Not found."})
        try:
            size = int(self.headers.get("Content-Length", 0))
            result = plan(json.loads(self.rfile.read(size) or b"{}"))
            self.reply(200, result)
        except ValueError as error:  # bad input or no route: the user can fix it
            status = 422 if "route" in str(error).lower() else 400
            self.reply(status, {"success": False, "error": str(error)})
        except Exception:
            self.reply(500, {"success": False, "error": "Unexpected server error."})

    def log_message(self, *args):
        pass  # keep the console quiet


def serve(port=5000, open_browser=True):
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    url = f"http://127.0.0.1:{port}"
    print(f"Drone planner running at {url}  (Ctrl+C to stop)")
    if open_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


# ------------------------------------------------------------------ tests
def run_tests():
    import urllib.request

    P = lambda x, y: {"x": x, "y": y}
    walk = lambda path: sum(math.dist(a, b) for a, b in zip(path, path[1:]))

    # both algorithms reach the goal, avoid obstacles and agree on distance
    blocked = set(DEFAULT_OBSTACLES)
    a, a_seen = find_path((2, 2), (45, 45), blocked, "astar")
    d, d_seen = find_path((2, 2), (45, 45), blocked, "dijkstra")
    assert a[0] == (2, 2) and a[-1] == (45, 45) and d[-1] == (45, 45)
    assert not (set(a) & blocked) and not (set(d) & blocked)
    assert abs(walk(a) - walk(d)) < 1e-9
    assert a_seen < d_seen  # A* searches fewer cells

    # straight line collapses to two waypoints; a turn keeps its corner
    assert keep_corners([(0, 0), (1, 0), (2, 0)]) == [(0, 0), (2, 0)]
    assert keep_corners([(0, 0), (1, 0), (2, 0), (2, 1)]) == [(0, 0), (2, 0), (2, 1)]

    # no squeezing diagonally between two touching obstacles
    global SIZE
    real_size, SIZE = SIZE, 2
    try:
        find_path((0, 0), (1, 1), {(1, 0), (0, 1)})
        raise AssertionError("should have failed")
    except ValueError:
        pass
    finally:
        SIZE = real_size

    # plan(): metrics and error handling
    r = plan({"start": P(0, 0), "destination": P(5, 0), "drone_speed": 10})
    assert r["distance_m"] == 50.0 and r["flight_time"] == 5.0 and r["waypoints"] == 2
    bad = [
        {},
        {"start": P(0, 0), "destination": P(99, 0)},
        {"start": P(0, 0), "destination": P(5, 5), "drone_speed": 0},
        {"start": P(0, 0), "destination": P(5, 5), "algorithm": "bfs"},
        {"start": P(5, 5), "destination": P(9, 9), "obstacles": [P(5, 5)]},
        {"start": P(0, 0), "destination": P(9, 0), "obstacles": [P(4, y) for y in range(SIZE)]},
    ]
    for request in bad:
        try:
            plan(request)
            raise AssertionError(f"accepted bad request: {request}")
        except ValueError:
            pass

    # the real HTTP server
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_port}"
    assert b"Drone" in urllib.request.urlopen(base + "/").read()
    zones = json.loads(urllib.request.urlopen(base + "/api/zones").read())
    assert len(zones["obstacles"]) == len(DEFAULT_OBSTACLES)
    req = urllib.request.Request(base + "/api/plan", json.dumps(
        {"start": P(2, 2), "destination": P(45, 45), "obstacles": zones["obstacles"]}).encode())
    assert json.loads(urllib.request.urlopen(req).read())["success"]
    server.shutdown()

    print("All tests passed.")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        run_tests()
    else:
        serve()
