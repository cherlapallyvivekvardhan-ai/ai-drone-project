import json
from pathlib import Path
from backend.models import MissionRequest
from backend.server import build_mission

def test_default_mission():
    zones = json.loads(Path("data/default_zones.json").read_text())["zones"]
    req = MissionRequest(
        start={"x":30,"y":40,"z":30},
        destination={"x":360,"y":330,"z":40},
        zones=zones
    )
    result = build_mission(req)
    assert result["status"] == "success"
    assert len(result["selected_route"]) >= 2
    assert result["selected_metrics"]["distance_m"] > 0
