import React, { useState, useCallback } from 'react';
import { useJsApiLoader } from '@react-google-maps/api';
import Header from './components/Header';
import MapView from './components/MapView';
import LocationSearch from './components/LocationSearch';
import RoutePanel from './components/RoutePanel';
import { calculateDroneRoute } from './services/api';

const libraries = ['places'];

export default function App() {
  const { isLoaded, loadError } = useJsApiLoader({
    googleMapsApiKey: import.meta.env.VITE_GOOGLE_MAPS_API_KEY || '',
    libraries,
  });

  const [start, setStart] = useState(null);
  const [destination, setDestination] = useState(null);
  const [routeData, setRouteData] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const handleMapClick = useCallback((event) => {
    const lat = event.latLng.lat();
    const lng = event.latLng.lng();

    // Alternate picking start and destination via map clicks
    if (!start) {
      setStart({ latitude: lat, longitude: lng });
    } else if (!destination) {
      setDestination({ latitude: lat, longitude: lng });
    }
  }, [start, destination]);

  const handleClear = () => {
    setStart(null);
    setDestination(null);
    setRouteData(null);
    setError(null);
  };

  const handleComputeRoute = async () => {
    if (!start || !destination) {
      setError('Both a Start and Destination location must be provided.');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const data = await calculateDroneRoute(start, destination);
      if (data.success) {
        setRouteData(data);
      } else {
        setError(data.message || 'Path planning failed.');
      }
    } catch (err) {
      const msg = err.response?.data?.detail || err.message || 'Failed to reach Python backend.';
      setError(`Path Planner Error: ${msg}`);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="app-container">
      <Header />
      <div className="main-content">
        <div className="sidebar">
          <div className="instructions">
            <p style={{ fontSize: '0.85rem', color: '#64748b' }}>
              Search using the inputs or tap the map directly to place origin and drop-off markers.
            </p>
          </div>

          <LocationSearch
            label="Departure Point (Base Hub)"
            placeholder="Search address or tap map"
            location={start}
            onLocationSelect={setStart}
            isLoaded={isLoaded}
          />

          <LocationSearch
            label="Dropoff Point (Delivery Site)"
            placeholder="Search address or tap map"
            location={destination}
            onLocationSelect={setDestination}
            isLoaded={isLoaded}
          />

          <div className="button-row">
            <button
              className="btn-primary"
              onClick={handleComputeRoute}
              disabled={isLoading || !start || !destination}
            >
              {isLoading ? 'Computing Path (A*)...' : 'Find Drone Path'}
            </button>
            <button className="btn-secondary" onClick={handleClear} disabled={isLoading}>
              Clear
            </button>
          </div>

          {error && <div className="error-box">{error}</div>}

          {loadError && (
            <div className="error-box">
              Google Maps failed to load. Check your VITE_GOOGLE_MAPS_API_KEY.
            </div>
          )}

          <RoutePanel routeData={routeData} />
        </div>

        <div className="map-pane">
          <MapView
            isLoaded={isLoaded}
            start={start}
            destination={destination}
            routeCoordinates={routeData?.route || []}
            onMapClick={handleMapClick}
          />
        </div>
      </div>
    </div>
  );
}
