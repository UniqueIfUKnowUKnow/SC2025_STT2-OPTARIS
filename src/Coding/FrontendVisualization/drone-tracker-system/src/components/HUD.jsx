// src/components/HUD.jsx
import React from 'react';
import useDroneStore from '../useDroneStore';

// --- Helper Components are now inside HUD.jsx ---

const ControlButton = ({ onClick, disabled, children, className }) => (
  <button onClick={onClick} disabled={disabled} className={className}>
    {children}
  </button>
);

const ControlPanel = ({ handleStart, handleStop, handleReset }) => {
  const status = useDroneStore((state) => state.status);
  const isRunning = !['STANDBY', 'PAUSED', 'ERROR', 'CONNECTING...'].includes(status);
  const canStart = status === 'STANDBY' || status === 'PAUSED';

  return (
    <div className="hud-panel">
      <h3>System Control</h3>
      <div className="control-buttons">
        <ControlButton onClick={handleStart} disabled={!canStart} className="start-btn">
          {status === 'PAUSED' ? 'Resume' : 'Start'}
        </ControlButton>
        <ControlButton onClick={handleStop} disabled={!isRunning} className="stop-btn">
          Stop
        </ControlButton>
        <ControlButton onClick={handleReset} disabled={status === 'STANDBY' || status === 'CONNECTING...'} className="reset-btn">
          Reset
        </ControlButton>
      </div>
    </div>
  );
};

const TelemetryItem = ({ label, value, unit = '' }) => (
  <div className="telemetry-item">
    <span>{label}</span>
    <strong>{value ?? 'N/A'}{unit}</strong>
  </div>
);


// --- Main HUD Component ---

export default function HUD({ handleStart, handleStop, handleReset }) {
  // Select the state safely
  const { status, progress, warning, error, payload } = useDroneStore();
  
  // Provide default values to prevent crashes if payload is not yet ready.
  const live_telemetry = payload?.live_telemetry || {};
  const position_history = payload?.position_history || [];
  const measured_pos = payload?.measured_pos;

  const latestHistory = [...position_history].reverse().slice(0, 20);

  return (
    <div className="hud">
      <div className="hud-left">
        <ControlPanel 
            handleStart={handleStart}
            handleStop={handleStop}
            handleReset={handleReset}
        />

        <div className="hud-panel">
          <h3>System Status</h3>
          <div className="status-line">
            <span>Status</span>
            <span className="status-value">{status}</span>
          </div>
          {warning && <div className="warning-text">Warning: {warning}</div>}
          {error && <div className="error-text">Error: {error}</div>}
          {/* Check if status exists before calling .includes() */}
          {(status && status.includes('PROGRESS')) && (
            <progress value={progress} max="100" />
          )}
        </div>

        <div className="hud-panel">
          <h3>Live Telemetry</h3>
          <TelemetryItem label="Distance" value={live_telemetry.distance} unit=" m" />
          <TelemetryItem label="Signal Strength" value={live_telemetry.signal_strength} unit=" %" />
          <TelemetryItem label="Pan / Tilt" value={live_telemetry.pan_angle ? `${live_telemetry.pan_angle}° / ${live_telemetry.tilt_angle}°` : 'N/A'} />
          <TelemetryItem label="Live XYZ" value={measured_pos?.[0] !== null ? measured_pos.join(', ') : 'N/A'} />
        </div>
        
        <div className="hud-panel">
            <h3>Legend</h3>
            <div className="legend-item"><span>🟢 Current Position</span></div>
            <div className="legend-item"><span>⚪ Previous Positions</span></div>
            <div className="legend-item"><span>🔵 Predicted Path</span></div>
            <div className="legend-item"><span>🔴 Laser Scanner</span></div>
        </div>
      </div>

      <div className="hud-panel hud-right">
        <h3>Position History (Last 20)</h3>
        <div className="history-table">
          <table>
            <thead>
              <tr>
                <th>#</th><th>X</th><th>Y</th><th>Z</th>
              </tr>
            </thead>
            <tbody>
              {latestHistory.map((item, index) => (
                <tr key={index}>
                  <td>{latestHistory.length - index}</td>
                  <td>{item.pos[0]}</td>
                  <td>{item.pos[1]}</td>
                  <td>{item.pos[2]}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
}