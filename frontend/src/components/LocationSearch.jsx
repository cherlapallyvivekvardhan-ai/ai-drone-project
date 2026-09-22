import React from 'react';
import { Autocomplete } from '@react-google-maps/api';

export default function LocationSearch({ label, placeholder, location, onLocationSelect, isLoaded }) {
  const autocompleteRef = React.useRef(null);

  const onLoad = (autoC) => {
    autocompleteRef.current = autoC;
  };

  const onPlaceChanged = () => {
    if (autocompleteRef.current !== null) {
      const place = autocompleteRef.current.getPlace();
      if (place.geometry && place.geometry.location) {
        const lat = place.geometry.location.lat();
        const lng = place.geometry.location.lng();
        onLocationSelect({ latitude: lat, longitude: lng });
      }
    }
  };

  return (
    <div className="form-group">
      <label>{label}</label>
      {isLoaded ? (
        <Autocomplete onLoad={onLoad} onPlaceChanged={onPlaceChanged}>
          <input
            type="text"
            className="form-input"
            placeholder={placeholder}
          />
        </Autocomplete>
      ) : (
        <input
          type="text"
          className="form-input"
          placeholder="Loading Google Places..."
          disabled
        />
      )}
      {location && (
        <span className="coord-display">
          Lat: {location.latitude.toFixed(5)}, Lng: {location.longitude.toFixed(5)}
        </span>
      )}
    </div>
  );
}
