# AeroPath AI — Delivery Drone Path Planning System

Full-stack academic project: a React + Google Maps frontend and a FastAPI
backend that computes A*-optimized delivery drone flight paths between a
base hub and a delivery site.

## Stack
- **Frontend:** React 18 + Vite, `@react-google-maps/api`, Axios
- **Backend:** FastAPI, Pydantic, A* pathfinding over a discretized grid
- **Database:** PostgreSQL + PostGIS (schema included; wiring up persistence
  is the natural next stage)
- **Orchestration:** Docker Compose

## Project Structure
See the full tree in this repo — `frontend/` (React app), `backend/`
(FastAPI app), `database/` (PostGIS schema).

## Quick Start — Docker Compose (backend + database)
```bash
cp backend/.env.example backend/.env
docker compose up --build
```
This starts:
- `db` on `localhost:5432` (Postgres + PostGIS)
- `backend` on `localhost:8000` (FastAPI, docs at `/docs`)

## Quick Start — Frontend (run separately)
```bash
cd frontend
cp .env.example .env   # add your VITE_GOOGLE_MAPS_API_KEY
npm install
npm run dev
```
Visit `http://localhost:5173`.

## Quick Start — Backend without Docker
```bash
cd backend
python -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install -r requirements.txt
cp .env.example .env
uvicorn main:app --reload --port 8000
```

## API
### `POST /api/route`
Request:
```json
{
  "start": { "latitude": 17.385, "longitude": 78.4867 },
  "destination": { "latitude": 17.412, "longitude": 78.4483 }
}
```
Response:
```json
{
  "success": true,
  "route": [ { "latitude": 17.385, "longitude": 78.4867 }, ... ],
  "distance_km": 5.42,
  "estimated_time_minutes": 8.1,
  "drone_speed_kmh": 40.0,
  "waypoints": 14,
  "battery_required_percent": 32.1,
  "message": "Route computed successfully."
}
```

### `GET /api/health`
Liveness check.

## How the pathfinding works
1. A bounding grid (default 25×25 cells, with padding) is built around the
   start/destination pair (`backend/utils/grid.py`).
2. A* (`backend/services/path_planner.py`) searches the grid with an
   8-directional movement model and a Euclidean heuristic. The `no_fly_zones`
   parameter is ready for extending with restricted airspace cells.
3. `backend/services/drone_optimizer.py` converts the resulting waypoint
   list into flight telemetry: distance (Haversine), flight time, and
   estimated battery consumption.

## Notes
- The Google Maps API key needs the **Maps JavaScript API** and **Places
  API** enabled.
- `BATTERY_RANGE_KM` and `DRONE_SPEED_KMH` in `drone_optimizer.py` are
  placeholder constants for a small multirotor — tune them to match your
  target drone spec.
