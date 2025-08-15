// src/components/HUD.jsx
// This file contains the Heads-Up Display (HUD) component that overlays
// controls, status information, and telemetry data on top of the 3D scene

// Import React for component creation
import React from 'react';
// Import our global state management store
import useDroneStore from '../useDroneStore';
import './HUD.css'; // Add a CSS import for custom layout

// --- Helper Components are now inside HUD.jsx ---

// Reusable button component for control panel actions
// Props: onClick function, disabled state, children (button text), and CSS class
const ControlButton = ({ onClick, disabled, children, className }) => (
  <button onClick={onClick} disabled={disabled} className={className}>
    {children}
  </button>
);

// Control panel component that contains Start, Stop, and Reset buttons
// Props: handler functions for each control action
const ControlPanel = ({ handleStart, handleStop, handleReset }) => {
  // Get current system status from the global store
  const status = useDroneStore((state) => state.status);
  
  // Determine if system is currently running (not in standby, paused, error, or connecting)
  const isRunning = status === 'CONNECTED' || status === 'TRACKING_TARGET' || status?.includes('IN_PROGRESS');
  
  // Determine if system can be started (always allow when not actively connected)
  const canStart = true;

  return (
    <div className="hud-panel">
      {/* Panel title */}
      <h3>System Control</h3>
      {/* Container for control buttons */}
      <div className="control-buttons">
        {/* Start/Resume button */}
        <ControlButton onClick={handleStart} disabled={!canStart} className="start-btn">
          Start
        </ControlButton>
        {/* Stop button - only enabled when system is running */}
        <ControlButton onClick={handleStop} disabled={!isRunning} className="stop-btn">
          Stop
        </ControlButton>
        {/* Reset button - always available to clear UI and request backend reset */}
        <ControlButton onClick={handleReset} className="reset-btn">
          Reset
        </ControlButton>
      </div>
    </div>
  );
};

// Reusable component for displaying telemetry data with label and value
// Props: label text, value to display, and optional unit (defaults to empty string)
const TelemetryItem = ({ label, value, unit = '' }) => (
  <div className="telemetry-item">
    {/* Display the label (e.g., "Distance", "Signal Strength") */}
    <span>{label}</span>
    {/* Display the value with unit, or "N/A" if value is null/undefined */}
    <strong>{value ?? 'N/A'}{unit}</strong>
  </div>
);


// --- Main HUD Component ---

// Helper to format numbers with units and fallback
const formatValue = (value, unit = '', digits = 2) => {
  if (value === null || value === undefined || isNaN(value)) return 'N/A';
  return `${Number(value).toFixed(digits)}${unit}`;
};

// Helper to extract classical orbital elements from initial_orbit_params
function getOrbitalElements(params) {
  if (!params) return {};
  return {
    a: params.a, // semi-major axis
    b: params.b, // semi-minor axis
    e: params.eccentricity, // eccentricity
    i: params.inclination, // inclination (deg)
    raan: params.raan, // right ascension of ascending node (deg)
    argPeriapsis: params.argPeriapsis, // argument of periapsis (deg)
  };
}

export default function HUD({ handleStart, handleStop, handleReset }) {
  const { status, progress, warning, error, payload } = useDroneStore();
  const live_telemetry = payload?.live_telemetry || {};
  const measured_pos = payload?.measured_pos;
  const initial_orbit_params = payload?.initial_orbit_params;
  const elements = getOrbitalElements(initial_orbit_params);

  return (
    <div className="hud-overlay">
      <div className="hud-panel hud-control">
        <h3>System Control</h3>
        <div className="control-buttons">
          <button onClick={handleStart} className="start-btn">Start</button>
          <button onClick={handleStop} className="stop-btn">Stop</button>
          <button onClick={handleReset} className="reset-btn">Reset</button>
        </div>
      </div>
      <div className="hud-panel hud-status">
        <h3>Status</h3>
        <div className="status-line">
          <span>Status</span>
          <span className="status-value">{status}</span>
        </div>
        {warning && <div className="warning-text">Warning: {warning}</div>}
        {error && <div className="error-text">Error: {error}</div>}
        {(status && status.includes('PROGRESS')) && (
          <progress value={progress} max="100" />
        )}
      </div>
      <div className="hud-panel hud-telemetry">
        <h3>Live Telemetry</h3>
        <div className="telemetry-item"><span>Distance</span><strong>{formatValue(live_telemetry.distance, ' m')}</strong></div>
        <div className="telemetry-item"><span>Signal Strength</span><strong>{formatValue(live_telemetry.signal_strength, ' %')}</strong></div>
        <div className="telemetry-item"><span>Pan / Tilt</span><strong>{live_telemetry.pan_angle !== undefined ? `${formatValue(live_telemetry.pan_angle, '°', 1)} / ${formatValue(live_telemetry.tilt_angle, '°', 1)}` : 'N/A'}</strong></div>
        <div className="telemetry-item"><span>Live XYZ</span><strong>{measured_pos?.[0] !== null ? measured_pos.join(', ') : 'N/A'}</strong></div>
      </div>
      <div className="hud-panel hud-orbit">
        <h3>Initial Orbit Parameters</h3>
        <div className="telemetry-item"><span>Semi-Major Axis (a)</span><strong>{formatValue(elements.a, ' km')}</strong></div>
        <div className="telemetry-item"><span>Semi-Minor Axis (b)</span><strong>{formatValue(elements.b, ' km')}</strong></div>
        <div className="telemetry-item"><span>Eccentricity (e)</span><strong>{formatValue(elements.e, '', 4)}</strong></div>
        <div className="telemetry-item"><span>Inclination (i)</span><strong>{formatValue(elements.i, '°', 2)}</strong></div>
        <div className="telemetry-item"><span>RAAN (Ω)</span><strong>{formatValue(elements.raan, '°', 2)}</strong></div>
        <div className="telemetry-item"><span>Arg. of Periapsis (ω)</span><strong>{formatValue(elements.argPeriapsis, '°', 2)}</strong></div>
      </div>
    </div>
  );
}