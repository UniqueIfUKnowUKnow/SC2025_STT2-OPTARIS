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
  appMode,                        // Current application mode ('simulation' or 'real')
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
  onReset,                       // Function to call when the Reset button is clicked
  onModeChange,                  // NEW: Function to call when mode changes
  onTestConnection               // NEW: Function to test WebSocket connection
}) => {
  // Extract orbital elements from the predicted orbit parameters
  const elements = getOrbitalElements(predictedOrbitParams);
  
  // Determine if the system is currently running based on the status
  // This is used to enable/disable the Stop button
  const isRunning = status === 'TRACKING_TARGET' || status?.includes('IN_PROGRESS') || status?.includes('SEARCHING');
  
  // State for the current date and time - this updates every second
  const [currentDateTime, setCurrentDateTime] = useState(new Date());
  
  // NEW: State for mode selection
  const [selectedMode, setSelectedMode] = useState(appMode || 'simulation');

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

  // Effect to sync selectedMode with appMode prop
  useEffect(() => {
    setSelectedMode(appMode || 'simulation');
  }, [appMode]);

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

  // NEW: Function to get mode-specific button states
  const getButtonStates = () => {
    if (selectedMode === 'real') {
      // In Real Mode, buttons are disabled as the Raspberry Pi controls the process
      return {
        startDisabled: true,
        stopDisabled: true,
        resetDisabled: true,
        startTooltip: 'In Real Mode, the scanning process is controlled by the Raspberry Pi',
        stopTooltip: 'In Real Mode, the scanning process is controlled by the Raspberry Pi',
        resetTooltip: 'In Real Mode, the scanning process is controlled by the Raspberry Pi'
      };
    } else {
      // In Simulation Mode, buttons work normally
      return {
        startDisabled: isRunning,
        stopDisabled: !isRunning,
        resetDisabled: false,
        startTooltip: 'Start the simulation',
        stopTooltip: 'Stop the simulation',
        resetTooltip: 'Reset all data and return to standby'
      };
    }
  };

  const buttonStates = getButtonStates();

  // This is what gets displayed on the screen
  return (
    <div className="system-hud">
      {/* Left Panel - System Status and Controls */}
      <div className="left-panel">
        
        {/* NEW: Mode Selection Panel */}
        <div className="mode-panel">
          <h3>Operation Mode</h3>
          <div className="mode-options">
            <label className="mode-option">
              <input
                type="radio"
                name="mode"
                value="simulation"
                checked={selectedMode === 'simulation'}
                onChange={(e) => {
                  setSelectedMode(e.target.value);
                  onModeChange?.(e.target.value);
                }}
              />
              <span className="mode-label">Simulation Mode</span>
              <span className="mode-description">Run with simulated data for testing and development</span>
            </label>
            
            <label className="mode-option">
              <input
                type="radio"
                name="mode"
                value="real"
                checked={selectedMode === 'real'}
                onChange={(e) => {
                  setSelectedMode(e.target.value);
                  onModeChange?.(e.target.value);
                }}
              />
              <span className="mode-label">Real Mode</span>
              <span className="mode-description">Connect to actual Raspberry Pi hardware for live data</span>
            </label>
          </div>
        </div>
        
        {/* System Status Panel - Shows current status, connection info, and WebSocket settings */}
        <div className="status-panel">
          <h3>System Status</h3>
          
          {/* Current Status */}
          <div className="status-item">
            <span className="status-label">Status:</span>
            <span className={`status-value ${status?.toLowerCase().replace(/\s+/g, '-')}`}>
              {status || 'STANDBY'}
            </span>
          </div>
          
          {/* Progress Bar */}
          {progress > 0 && (
            <div className="progress-item">
              <span className="progress-label">Progress:</span>
              <div className="progress-bar">
                <div 
                  className="progress-fill" 
                  style={{ width: `${progress}%` }}
                />
              </div>
              <span className="progress-text">{progress}%</span>
            </div>
          )}
          
          {/* NEW: Real Mode Status Display */}
          {selectedMode === 'real' && (
            <div className="real-mode-status">
              <div className="status-item">
                <span className="status-label">WebSocket:</span>
                <span className={`status-value ${wsStatus}`}>
                  {wsStatus === 'connected' ? 'Connected' : 
                   wsStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
                </span>
              </div>
              
              {/* WebSocket URL input field and Connect button */}
              <div className="websocket-config">
                <label htmlFor="wsUrl">WebSocket URL:</label>
                <input
                  id="wsUrl"
                  type="text"
                  value={wsUrl}
                  onChange={(e) => onWsUrlChange?.(e.target.value)}
                  placeholder="ws://192.168.1.100:8765"
                  aria-label="WebSocket connection URL"
                />
                <div className="websocket-buttons">
                  <button 
                    className="connect-button"
                    onClick={onReconnect}
                    disabled={wsStatus === 'connecting'}
                  >
                    {wsStatus === 'connecting' ? 'Connecting...' : 'Connect'}
                  </button>
                  <button 
                    className="test-button"
                    onClick={onTestConnection}
                    disabled={wsStatus === 'connecting'}
                    title="Test connection without establishing full connection"
                  >
                    Test
                  </button>
                </div>
              </div>
            </div>
          )}
          
          {/* Warnings and Errors */}
          {warning && (
            <div className="warning-item">
              <span className="warning-icon">⚠️</span>
              <span className="warning-text">{warning}</span>
            </div>
          )}
          
          {error && (
            <div className="error-item">
              <span className="error-icon">❌</span>
              <span className="error-text">{error}</span>
            </div>
          )}
        </div>
        
        {/* Control Panel - Buttons for controlling the system */}
        <div className="control-panel">
          <h3>Controls</h3>
          
          {/* NEW: Mode-specific control buttons */}
          <div className="control-buttons">
            <ControlButton
              onClick={onStart}
              disabled={buttonStates.startDisabled}
              className="control-button start-button"
              title={buttonStates.startTooltip}
            >
              Start
            </ControlButton>
            
            <ControlButton
              onClick={onStop}
              disabled={buttonStates.stopDisabled}
              className="control-button stop-button"
              title={buttonStates.stopTooltip}
            >
              Stop
            </ControlButton>
            
            <ControlButton
              onClick={onReset}
              disabled={buttonStates.resetDisabled}
              className="control-button reset-button"
              title={buttonStates.resetTooltip}
            >
              Reset
            </ControlButton>
          </div>
          
          {/* View switching */}
          <div className="view-controls">
            <button 
              className={`view-button ${currentView === 'global' ? 'active' : ''}`}
              onClick={() => onSwitchView()}
            >
              Global View
            </button>
            <button 
              className={`view-button ${currentView === 'local' ? 'active' : ''}`}
              onClick={() => onSwitchView()}
            >
              Local View
            </button>
          </div>
        </div>
      </div>

      {/* Right Panel - Data Display and Telemetry */}
      <div className="right-panel">
        
        {/* Position Data Panel */}
        <div className="data-panel">
          <h3>Position Data</h3>
          
          {/* Measured Position */}
          <div className="data-item">
            <span className="data-label">Measured Position:</span>
            <span className="data-value">
              {measuredPosition ? 
                `X: ${formatValue(measuredPosition[0], 'm')}, Y: ${formatValue(measuredPosition[1], 'm')}, Z: ${formatValue(measuredPosition[2], 'm')}` : 
                'No data'
              }
            </span>
          </div>
          
          {/* NEW: Enhanced Live Telemetry for Real Mode */}
          <div className="telemetry-section">
            <h4>Live Telemetry</h4>
            <div className="telemetry-grid">
              <div className="telemetry-item">
                <span className="telemetry-label">Pan Angle:</span>
                <span className="telemetry-value">
                  {formatValue(liveTelemetry?.pan_angle, '°')}
                </span>
              </div>
              <div className="telemetry-item">
                <span className="telemetry-label">Tilt Angle:</span>
                <span className="telemetry-value">
                  {formatValue(liveTelemetry?.tilt_angle, '°')}
                </span>
              </div>
              <div className="telemetry-item">
                <span className="telemetry-label">Distance:</span>
                <span className="telemetry-value">
                  {formatValue(liveTelemetry?.distance, 'm')}
                </span>
              </div>
              <div className="telemetry-item">
                <span className="telemetry-label">Signal Strength:</span>
                <span className="telemetry-value">
                  {formatValue(liveTelemetry?.signal_strength, '%')}
                </span>
              </div>
            </div>
          </div>
        </div>
        
        {/* Orbital Parameters Panel */}
        {predictedOrbitParams && (
          <div className="data-panel">
            <h3>Orbital Parameters</h3>
            
            {/* Semi-major Axis */}
            {elements.semiMajorAxis && (
              <div className="data-item">
                <span className="data-label">Semi-major Axis:</span>
                <span className="data-value">
                  {formatValue(elements.semiMajorAxis, ' km')}
                </span>
              </div>
            )}
            
            {/* Eccentricity */}
            {elements.eccentricity && (
              <div className="data-item">
                <span className="data-label">Eccentricity:</span>
                <span className="data-value">
                  {formatValue(elements.eccentricity, '', 4)}
                </span>
              </div>
            )}
            
            {/* Inclination */}
            {elements.inclination && (
              <div className="data-item">
                <span className="data-label">Inclination:</span>
                <span className="data-value">
                  {formatValue(elements.inclination, '°')}
                </span>
              </div>
            )}
            
            {/* Argument of Perigee */}
            {elements.argumentOfPerigee && (
              <div className="data-item">
                <span className="data-label">Argument of Perigee:</span>
                <span className="data-value">
                  {formatValue(elements.argumentOfPerigee, '°')}
                </span>
              </div>
            )}
            
            {/* Right Ascension of Ascending Node */}
            {elements.rightAscensionOfAscendingNode && (
              <div className="data-item">
                <span className="data-label">RAAN:</span>
                <span className="data-value">
                  {formatValue(elements.rightAscensionOfAscendingNode, '°')}
                </span>
              </div>
            )}
            
            {/* Mean Anomaly */}
            {elements.meanAnomaly && (
              <div className="data-item">
                <span className="data-label">Mean Anomaly:</span>
                <span className="data-value">
                  {formatValue(elements.meanAnomaly, '°')}
                </span>
              </div>
            )}
          </div>
        )}
        
        {/* Date and Time Panel */}
        <div className="data-panel">
          <h3>Date & Time</h3>
          <div className="data-item">
            <span className="data-label">Date:</span>
            <span className="data-value">{formatDate(currentDateTime)}</span>
          </div>
          <div className="data-item">
            <span className="data-label">Time:</span>
            <span className="data-value">{formatTime(currentDateTime)}</span>
          </div>
        </div>
      </div>
    </div>
  );
};

// Export the SystemHUD component so it can be used by the main App component
export default SystemHUD;
