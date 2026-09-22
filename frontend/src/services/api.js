import axios from 'axios';

const API_BASE_URL = import.meta.env.VITE_API_BASE_URL || 'http://localhost:8000';

const client = axios.create({
  baseURL: API_BASE_URL,
  headers: {
    'Content-Type': 'application/json',
  },
  timeout: 10000,
});

export const calculateDroneRoute = async (start, destination) => {
  const payload = {
    start: {
      latitude: parseFloat(start.latitude),
      longitude: parseFloat(start.longitude)
    },
    destination: {
      latitude: parseFloat(destination.latitude),
      longitude: parseFloat(destination.longitude)
    }
  };

  const response = await client.post('/api/route', payload);
  return response.data;
};
