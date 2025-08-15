import React, { useState, useEffect } from 'react';
import './SystemHUD.css';

// SystemHUD Component - This is the main interface overlay that shows all the control panels and information
// The panels are positioned on the left and right sides of the screen and remain stationary when users interact with the 3D scene

// Helper function to format numbers with units (e.g., "123.45 km" or "67.89°")
// This makes the data more readable for users
const formatValue = (value, unit = '', digits = 2) => {
  // If the value is null, undefined, or not a number, show "N/A" (Not Available)
  if (value === null || value === undefined || isNaN(value)) return 'N/A';
  
  // Convert the value to a number and format it to the specified number of decimal places
  // Then add the unit (like "km" for kilometers or "°" for degrees)
  return `${Number(value).toFixed(digits)}${unit}`;
};

// Helper function to extract orbital elements from the predicted orbit parameters
// Orbital elements describe the shape, size, and orientation of a satellite's orbit
function getOrbitalElements(params) {
  // If there are no parameters or no elements, return an empty object
  if (!params || !params.elements) return {};
  
  // Return the orbital elements (like semi-major axis, eccentricity, inclination, etc.)
  return params.elements;
}

// Control Button Component - This creates consistent-looking buttons throughout the interface
// Each button has the same styling and behavior patterns
const ControlButton = ({ onClick, disabled, children, className }) => (
  <button 
    onClick={onClick} 
    disabled={disabled} 
    className={className}
    // This makes the button accessible to screen readers and keyboard navigation
    aria-label={children}
  >
    {children}
  </button>
);

// Main SystemHUD component - This renders all the control panels and information displays
const SystemHUD = ({
  currentView,                    // Which view we're currently showing ('global' or 'local')
  status,                        // Current system status (e.g., "TRACKING_TARGET", "SEARCHING")
  progress,                      // Progress percentage for operations (0-100%)
  warning,                       // Any warning messages from the system
  error,                         // Any error messages from the system
  measuredPosition,              // The current position measured by the LiDAR sensor
  predictedOrbitParams,          // Calculated orbital parameters (like orbit shape, size, etc.)
  liveTelemetry,                 // Real-time data from the sensors (angles, signal strength, etc.)
  wsStatus,                      // WebSocket connection status ('connected', 'connecting', 'disconnected')
  wsUrl,                         // Current WebSocket connection URL
  onWsUrlChange,                 // Function to call when the WebSocket URL changes
  onReconnect,                   // Function to call when the Connect button is clicked
  onSwitchView,                  // Function to call when switching between global and local views
  onStart,                       // Function to call when the Start button is clicked
  onStop,                        // Function to call when the Stop button is clicked
  onReset                        // Function to call when the Reset button is clicked
}) => {
  // Extract orbital elements from the predicted orbit parameters
  const elements = getOrbitalElements(predictedOrbitParams);
  
  // Determine if the system is currently running based on the status
  // This is used to enable/disable the Stop button
  const isRunning = status === 'TRACKING_TARGET' || status?.includes('IN_PROGRESS') || status?.includes('SEARCHING');
  
  // State for the current date and time - this updates every second
  const [currentDateTime, setCurrentDateTime] = useState(new Date());

  // Effect to update the date and time every second
  // This creates a live clock that users can always see
  useEffect(() => {
    // Create a timer that updates every 1000 milliseconds (1 second)
    const timer = setInterval(() => {
      setCurrentDateTime(new Date());
    }, 1000);

    // Cleanup function - this runs when the component is removed from the screen
    // It stops the timer so we don't waste computer resources
    return () => clearInterval(timer);
  }, []);

  // Function to format the date in a readable way (e.g., "January 15, 2024")
  const formatDate = (date) => {
    return date.toLocaleDateString('en-US', {
      weekday: 'long',      // Full day name (e.g., "Monday")
      year: 'numeric',      // Full year (e.g., "2024")
      month: 'long',        // Full month name (e.g., "January")
      day: 'numeric'        // Day of the month (e.g., "15")
    });
  };

  // Function to format the time in a readable way (e.g., "2:30:45 PM")
  const formatTime = (date) => {
    return date.toLocaleTimeString('en-US', {
      hour: 'numeric',      // Hour (e.g., "2")
      minute: '2-digit',    // Minute with leading zero (e.g., "30")
      second: '2-digit',    // Second with leading zero (e.g., "45")
      hour12: true          // Use 12-hour format with AM/PM
    });
  };

  // This is what gets displayed on the screen
  return (
    <>
      {/* Main HUD Overlay - This contains all the control panels and information displays */}
      <div className="hud-overlay">
        
        {/* LEFT SIDE PANELS - These are positioned on the left side of the screen */}
        <div className="left-panels">
          
          {/* System Control Panel - Contains Start, Stop, Reset, and View Switch buttons */}
          <div className="hud-panel hud-control">
            <h3>System Control</h3>
            
            {/* Control buttons for starting, stopping, and resetting the system */}
            <div className="control-buttons">
              <ControlButton onClick={onStart} className="start-btn">
                Start
              </ControlButton>
              <ControlButton onClick={onStop} disabled={!isRunning} className="stop-btn">
                Stop
              </ControlButton>
              <ControlButton onClick={onReset} className="reset-btn">
                Reset
              </ControlButton>
            </div>
            
            {/* View switching button - allows users to switch between global and local views */}
            <div className="view-switch">
              <ControlButton onClick={onSwitchView} className="switch-btn">
                Switch to {currentView === 'global' ? 'Local' : 'Global'} View
              </ControlButton>
            </div>
          </div>

          {/* System Status Panel - Shows current status, connection info, and WebSocket settings */}
          <div className="hud-panel hud-status">
            <h3>System Status</h3>
            
            {/* Current system status display */}
            <div className="status-line">
              <span>Status:</span>
              <span className="status-value">{status || 'UNKNOWN'}</span>
            </div>
            
            {/* WebSocket connection status */}
            <div className="status-line">
              <span>Connection:</span>
              <span className={`status-value ${wsStatus === 'connected' ? 'connected' : 'disconnected'}`}>
                {wsStatus}
              </span>
            </div>
            
            {/* WebSocket URL input field and Connect button */}
            {/* This allows users to change the connection URL and reconnect */}
            <div className="ws-url-row">
              <span className="ws-url-label">WS URL:</span>
              <input
                className="ws-url-input"
                value={wsUrl}
                onChange={(e) => onWsUrlChange?.(e.target.value)}
                placeholder="ws://192.168.1.100:8765"
                // This makes the input field accessible to screen readers
                aria-label="WebSocket connection URL"
              />
              <button className="connect-btn" onClick={onReconnect}>Connect</button>
            </div>
            
            {/* Warning messages from the system */}
            {warning && (
              <div className="warning-text">Warning: {warning}</div>
            )}
            
            {/* Error messages from the system */}
            {error && (
              <div className="error-text">Error: {error}</div>
            )}
            
            {/* Progress bar for operations that take time */}
            {(status && status.includes('PROGRESS')) && (
              <div className="progress-container">
                <progress value={progress} max="100" />
                <span>{progress}%</span>
              </div>
            )}
          </div>

          {/* Target Data Panel - Shows current position and sensor readings */}
          <div className="hud-panel hud-data">
            <h3>Target Data</h3>
            
            {/* Current target position in 3D coordinates */}
            <div className="data-item">
              <span>Current Position:</span>
              <span className="data-value">
                {measuredPosition 
                  ? `${formatValue(measuredPosition[0], ' km')}, ${formatValue(measuredPosition[1], ' km')}, ${formatValue(measuredPosition[2], ' km')}`
                  : 'N/A'
                }
              </span>
            </div>
            
            {/* Distance to the target */}
            <div className="data-item">
              <span>Distance:</span>
              <span className="data-value">
                {formatValue(liveTelemetry?.distance, ' m')}
              </span>
            </div>
            
            {/* Signal strength from the LiDAR sensor */}
            <div className="data-item">
              <span>Signal Strength:</span>
              <span className="data-value">
                {formatValue(liveTelemetry?.signal_strength, ' %')}
              </span>
            </div>
          </div>
        </div>

        {/* RIGHT SIDE PANELS - These are positioned on the right side of the screen */}
        <div className="right-panels">
          
          {/* Orbital Parameters Panel - Shows calculated orbital elements */}
          <div className="hud-panel hud-orbit">
            <h3>Orbital Parameters</h3>
            
            {/* Semi-Major Axis - the "size" of the orbit */}
            <div className="orbit-item">
              <span>Semi-Major Axis (a):</span>
              <span className="orbit-value">
                {formatValue(elements.semiMajorAxis, ' km')}
              </span>
            </div>
            
            {/* Eccentricity - how "oval" the orbit is (0 = perfect circle, 1 = very elongated) */}
            <div className="orbit-item">
              <span>Eccentricity (e):</span>
              <span className="orbit-value">
                {formatValue(elements.eccentricity, '', 4)}
              </span>
            </div>
            
            {/* Inclination - how tilted the orbit is relative to Earth's equator */}
            <div className="orbit-item">
              <span>Inclination (i):</span>
              <span className="orbit-value">
                {formatValue(elements.inclinationDeg, '°')}
              </span>
            </div>
            
            {/* RAAN (Right Ascension of Ascending Node) - where the orbit crosses the equator going up */}
            <div className="orbit-item">
              <span>RAAN (Ω):</span>
              <span className="orbit-value">
                {formatValue(elements.raanDeg, '°')}
              </span>
            </div>
            
            {/* Argument of Periapsis - where the closest point to Earth is in the orbit */}
            <div className="orbit-item">
              <span>Arg. of Periapsis (ω):</span>
              <span className="orbit-value">
                {formatValue(elements.argPeriapsisDeg, '°')}
              </span>
            </div>
            
            {/* True Anomaly - current position in the orbit */}
            <div className="orbit-item">
              <span>True Anomaly (ν):</span>
              <span className="orbit-value">
                {formatValue(elements.trueAnomalyDeg, '°')}
              </span>
            </div>
          </div>

          {/* Visual Legend Panel - Explains what the different colors and symbols mean */}
          {/* The content changes automatically based on which view the user is looking at */}
          <div className="hud-panel hud-legend">
            <h3>Visual Legend</h3>
            
            {/* Legend for Global View (Earth and satellites) */}
            {currentView === 'global' ? (
              <>
                <div className="legend-item">
                  <span style={{ color: '#ff4444' }}>■</span> Sofia Tech Park (Ground Station)
                </div>
                <div className="legend-item">
                  <span style={{ color: '#ff6b35' }}>■</span> Optaris Satellite
                </div>
                <div className="legend-item">
                  <span style={{ color: '#00ff88' }}>■</span> ISS Satellite
                </div>
                <div className="legend-item">
                  <span style={{ color: '#4e9cff' }}>●</span> Measured Position (LiDAR)
                </div>
                <div className="legend-item">
                  <span style={{ color: '#ff6b35' }}>─</span> Predicted Orbit (Optaris)
                </div>
                <div className="legend-item">
                  <span style={{ color: '#00ff88' }}>─</span> ISS Orbit Path
                </div>
              </>
            ) : (
              // Legend for Local View (LiDAR tracker)
              <>
                <div className="legend-item">
                  <span style={{ color: 'red' }}>■</span> LiDAR Sensor
                </div>
                <div className="legend-item">
                  <span style={{ color: 'lime' }}>●</span> Target Position
                </div>
                <div className="legend-item">
                  <span style={{ color: 'white' }}>●</span> History (fades)
                </div>
                <div className="legend-item">
                  <span style={{ color: 'cyan' }}>─</span> Scanner Beam
                </div>
                <div className="legend-item">
                  <span style={{ color: 'red' }}>─</span> Tracking Beam
                </div>
                <div className="legend-item">
                  <span style={{ color: 'cyan' }}>○</span> 12m Range Sphere
                </div>
              </>
            )}
          </div>
        </div>
      </div>

      {/* Date and Time Display - Fixed at the bottom of the screen */}
      {/* This shows the current date and time, updating every second */}
      <div className="datetime-overlay">
        <div className="date-display">
          {formatDate(currentDateTime)}
        </div>
        <div className="time-display">
          {formatTime(currentDateTime)}
        </div>
      </div>
    </>
  );
};

// Export the SystemHUD component so it can be used by the main App component
export default SystemHUD;
