from fastapi import APIRouter, HTTPException

from models.schemas import RouteRequest, RouteResponse
from services.path_planner import plan_path
from services.drone_optimizer import compute_route_metrics, BATTERY_RANGE_KM

router = APIRouter()


@router.post("/route", response_model=RouteResponse)
def compute_route(request: RouteRequest) -> RouteResponse:
    """
    Plans an A*-optimized flight path between a start hub and a delivery
    destination, then returns the path plus estimated flight telemetry.
    """
    try:
        route = plan_path(request.start, request.destination)
        metrics = compute_route_metrics(route)

        if metrics["battery_required_percent"] >= 100.0:
            message = (
                f"Route exceeds drone range (~{BATTERY_RANGE_KM:.0f} km). "
                "Consider an intermediate charging hub."
            )
        else:
            message = "Route computed successfully."

        return RouteResponse(
            success=True,
            route=route,
            message=message,
            **metrics,
        )
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=500, detail=f"Path planning failed: {exc}") from exc


@router.get("/health")
def health_check() -> dict:
    """Simple liveness check used by docker-compose / uptime monitors."""
    return {"status": "ok"}
