import React from 'react';

export default function Header() {
  return (
    <header className="app-header">
      <div className="brand">
        <span className="brand-icon">🛸</span>
        <h1>AeroPath AI | Delivery Drone Routing</h1>
      </div>
      <div className="badge">
        Academic Core Engine: A* Search
      </div>
    </header>
  );
}
