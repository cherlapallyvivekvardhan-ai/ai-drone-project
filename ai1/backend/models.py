from typing import List, Optional
from pydantic import BaseModel, Field, ConfigDict

class Point3D(BaseModel):
    model_config = ConfigDict(extra="ignore")
    x: float
    y: float
    z: float = 0.0

class Wind(BaseModel):
    model_config = ConfigDict(extra="ignore")
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

class DroneConfig(BaseModel):
    model_config = ConfigDict(extra="ignore")
    cruise_speed_mps: float = Field(12.0, gt=0)
    battery_wh: float = Field(250.0, gt=0)
    base_power_w: float = Field(180.0, gt=0)
    climb_power_w_per_mps: float = Field(35.0, ge=0)
    wind_sensitivity: float = Field(0.35, ge=0)
    safety_margin: float = Field(1.25, ge=1.0)

class GridConfig(BaseModel):
    model_config = ConfigDict(extra="ignore")
    size_x: int = Field(31, ge=5, le=101)
    size_y: int = Field(31, ge=5, le=101)
    size_z: int = Field(9, ge=1, le=31)
    cell_size_m: float = Field(10.0, gt=0)

class Zone(BaseModel):
    model_config = ConfigDict(extra="ignore")
    id: str
    name: str = ""
    type: str = "obstacle"
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float = 0.0
    max_z: float = 1000.0

class MissionRequest(BaseModel):
    model_config = ConfigDict(extra="ignore")
    start: Point3D
    destination: Point3D
    zones: List[Zone] = []
    grid: GridConfig = GridConfig()
    drone: DroneConfig = DroneConfig()
    wind: Wind = Wind()
    algorithm: str = "astar"
    optimize: bool = True
