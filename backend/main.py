import os

from dotenv import load_dotenv
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from api.routes import router as route_router

load_dotenv()

FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:5173")

app = FastAPI(
    title="AeroPath AI - Drone Path Planning API",
    description="FastAPI backend that computes A*-optimized delivery drone flight paths.",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[FRONTEND_URL],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(route_router, prefix="/api", tags=["routing"])


@app.get("/")
def root() -> dict:
    return {
        "service": "AeroPath AI Drone Path Planning API",
        "status": "running",
        "docs": "/docs",
    }
