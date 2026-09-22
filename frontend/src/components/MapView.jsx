import React, { useMemo, useCallback } from 'react';
import { GoogleMap, Marker, Polyline } from '@react-google-maps/api';

const mapContainerStyle = {
  width: '100%',
  height: '100%',
};

const defaultCenter = {
  lat: 17.385044, // Default fallback (e.g. Hyderabad, India)
  lng: 78.486671,
};

const mapOptions = {
  disableDefaultUI: false,
  zoomControl: true,
  mapTypeControl: false,
  streetViewControl: false,
  fullscreenControl: true,
};

export default function MapView({
  isLoaded,
  start,
  destination,
  routeCoordinates,
  onMapClick,
}) {
  const [map, setMap] = React.useState(null);

  const onLoad = useCallback((mapInstance) => {
    setMap(mapInstance);
  }, []);

  const onUnmount = useCallback(() => {
    setMap(null);
  }, []);

  // Format points for the Google Maps Polyline
  const polylinePath = useMemo(() => {
    if (!routeCoordinates || routeCoordinates.length === 0) return [];
    return routeCoordinates.map((coord) => ({
      lat: coord.latitude,
      lng: coord.longitude,
    }));
  }, [routeCoordinates]);

  // Dynamically bound map viewport around points
  React.useEffect(() => {
    if (map && (start || destination)) {
      const bounds = new window.google.maps.LatLngBounds();
      if (start) bounds.extend(new window.google.maps.LatLng(start.latitude, start.longitude));
      if (destination) bounds.extend(new window.google.maps.LatLng(destination.latitude, destination.longitude));
      if (start && destination) {
        map.fitBounds(bounds, 80);
      } else if (start) {
        map.panTo(new window.google.maps.LatLng(start.latitude, start.longitude));
        map.setZoom(13);
      }
    }
  }, [map, start, destination]);

  if (!isLoaded) {
    return <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: '100%' }}>Loading Google Maps...</div>;
  }

  return (
    <GoogleMap
      mapContainerStyle={mapContainerStyle}
      center={start ? { lat: start.latitude, lng: start.longitude } : defaultCenter}
      zoom={12}
      onLoad={onLoad}
      onUnmount={onUnmount}
      onClick={onMapClick}
      options={mapOptions}
    >
      {/* Start Marker (Green Pin) */}
      {start && (
        <Marker
          position={{ lat: start.latitude, lng: start.longitude }}
          label={{ text: 'S', color: '#ffffff', fontWeight: 'bold' }}
          title="Start Station"
        />
      )}

      {/* Destination Marker (Red Pin) */}
      {destination && (
        <Marker
          position={{ lat: destination.latitude, lng: destination.longitude }}
          label={{ text: 'D', color: '#ffffff', fontWeight: 'bold' }}
          title="Delivery Target"
        />
      )}

      {/* Autonomous Drone Trajectory Vector */}
      {polylinePath.length > 0 && (
        <Polyline
          path={polylinePath}
          options={{
            strokeColor: '#0284c7',
            strokeOpacity: 0.9,
            strokeWeight: 4,
            geodesic: true,
          }}
        />
      )}
    </GoogleMap>
  );
}
