-- Enable PostGIS for geographic types (image already ships with the extension)
CREATE EXTENSION IF NOT EXISTS postgis;

CREATE TABLE IF NOT EXISTS routes (
    id SERIAL PRIMARY KEY,
    start_latitude DOUBLE PRECISION NOT NULL,
    start_longitude DOUBLE PRECISION NOT NULL,
    destination_latitude DOUBLE PRECISION NOT NULL,
    destination_longitude DOUBLE PRECISION NOT NULL,
    distance_km DOUBLE PRECISION NOT NULL,
    estimated_time_minutes DOUBLE PRECISION NOT NULL,
    drone_speed_kmh DOUBLE PRECISION NOT NULL,
    waypoints INTEGER NOT NULL,
    battery_required_percent DOUBLE PRECISION NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_routes_created_at ON routes (created_at DESC);
