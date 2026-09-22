"""
AI-Based Delivery Drone Path Planning System
Main entry point: FastAPI API + static frontend.

Run:
    python -m pip install -r requirements.txt
    python app.py
Open:
    http://127.0.0.1:8000
API docs:
    http://127.0.0.1:8000/docs
"""
from pathlib import Path
import json
import math
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from backend.models import MissionRequest
from backend.server import build_mission, load_zones_file

BASE = Path(__file__).resolve().parent
DATA = BASE / "data"
FRONTEND = BASE / "frontend"

app = FastAPI(
    title="AI Delivery Drone Path Planning System",
    version="1.0.0",
    description="3D grid A*, Dijkstra baseline, obstacle avoidance, battery/wind optimization."
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/frontend", StaticFiles(directory=str(FRONTEND)), name="frontend")

@app.get("/", include_in_schema=False)
def home():
    from fastapi.responses import FileResponse
    return FileResponse(FRONTEND / "index.html")

@app.get("/api/health")
def health():
    return {"status": "ok", "service": "drone_mission_platform"}

@app.get("/api/zones")
def zones():
    path = DATA / "default_zones.json"
    return json.loads(path.read_text(encoding="utf-8"))

@app.post("/api/upload-map")
async def upload_map(file: UploadFile = File(...)):
    if not file.filename or not file.filename.lower().endswith(".json"):
        raise HTTPException(400, "Upload a JSON/GeoJSON file.")
    raw = await file.read()
    if len(raw) > 5_000_000:
        raise HTTPException(413, "Map file is larger than 5 MB.")
    try:
        obj = json.loads(raw.decode("utf-8"))
        # Accept either the platform schema or GeoJSON FeatureCollection.
        zones = load_zones_file(obj)
        return {"filename": file.filename, "zones": zones}
    except Exception as exc:
        raise HTTPException(400, f"Invalid map JSON: {exc}")

@app.post("/api/plan")
def plan(req: MissionRequest):
    try:
        return build_mission(req)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except Exception as exc:
        raise HTTPException(500, f"Planner error: {exc}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app:app", host="127.0.0.1", port=8000, reload=False)
