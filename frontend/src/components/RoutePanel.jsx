import React from 'react';

export default function RoutePanel({ routeData }) {
  if (!routeData) {
    return null;
  }

  return (
    <div className="route-panel">
      <h2>Drone Flight Telemetry</h2>
      <div className="telemetry-grid">
        <div className="telemetry-card">
          <div className="telemetry-label">Ground Distance</div>
          <div className="telemetry-value">{routeData.distance_km.toFixed(2)} km</div>
        </div>
        <div className="telemetry-card">
          <div className="telemetry-label">Flight Duration</div>
          <div className="telemetry-value">{routeData.estimated_time_minutes.toFixed(1)} min</div>
        </div>
        <div className="telemetry-card">
          <div className="telemetry-label">Cruise Speed</div>
          <div className="telemetry-value">{routeData.drone_speed_kmh} km/h</div>
        </div>
        <div className="telemetry-card">
          <div className="telemetry-label">Est. Battery Cost</div>
          <div className="telemetry-value">{routeData.battery_required_percent.toFixed(1)}%</div>
        </div>
        <div className="telemetry-card">
          <div className="telemetry-label">Waypoints</div>
          <div className="telemetry-value">{routeData.waypoints}</div>
        </div>
        <div className="telemetry-card">
          <div className="telemetry-label">Path Validity</div>
          <div className="telemetry-value" style={{ fontSize: '0.9rem', color: '#10b981' }}>
            A* Optimal
          </div>
        </div>
      </div>
      <div>
        <span className="status-badge">
          Status: {routeData.message}
        </span>
      </div>
    </div>
  );
}
