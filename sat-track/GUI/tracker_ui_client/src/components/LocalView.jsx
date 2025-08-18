import React, { useRef, useEffect, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { Grid, Line, Html } from '@react-three/drei';
import * as THREE from 'three';
import ErrorBoundary from './ErrorBoundary';
import PredictedOrbit from './PredictedOrbit';

// LocalView Component - This renders the 3D LiDAR tracker view
// Users can see the LiDAR sensor, scanner beam, target position, and predicted orbit

// Constants for the local view scene
const LIDAR_RANGE_METERS = 12;        // Maximum range of the LiDAR sensor in meters
const SCENE_SCALE = 1;                // Scale factor for the 3D scene (1 unit = 1 meter)
const HISTORY_POINTS_MAX = 5;         // Maximum number of history points to show
const FADE_DURATION_MS = 5000;        // How long history points take to fade out (5 seconds)

// NEW: Real-Time Status Display Component
const RealTimeStatus = ({ status, progress, message, liveTelemetry }) => (
  <Html position={[0, 15, 0]} center>
    <div className="real-time-status-overlay">
      <div className="status-header">
        <h3>LiDAR Scanner Status</h3>
        <div className={`status-badge ${status?.toLowerCase().replace(/\s+/g, '-')}`}>
          {status || 'STANDBY'}
        </div>
      </div>
      
      {progress > 0 && (
        <div className="progress-section">
          <div className="progress-bar">
            <div 
              className="progress-fill" 
              style={{ width: `${progress}%` }}
            />
          </div>
          <span className="progress-text">{progress}%</span>
        </div>
      )}
      
      {message && (
        <div className="status-message">
          {message}
        </div>
      )}
      
      {liveTelemetry && (
        <div className="telemetry-display">
          <div className="telemetry-item">
            <span className="telemetry-label">Distance:</span>
            <span className="telemetry-value">
              {liveTelemetry.distance ? `${liveTelemetry.distance} cm` : 'N/A'}
            </span>
          </div>
          <div className="telemetry-item">
            <span className="telemetry-label">Pan Angle:</span>
            <span className="telemetry-value">
              {liveTelemetry.pan_angle ? `${liveTelemetry.pan_angle}°` : 'N/A'}
            </span>
          </div>
          <div className="telemetry-item">
            <span className="telemetry-label">Tilt Angle:</span>
            <span className="telemetry-value">
              {liveTelemetry.tilt_angle ? `${liveTelemetry.tilt_angle}°` : 'N/A'}
            </span>
          </div>
        </div>
      )}
    </div>
  </Html>
);

// History Point Component - This shows previous target positions as fading dots
// Each point represents where the target was measured at a previous time
const HistoryPoint = ({ position, timestamp, index }) => {
  // Calculate how much this point should fade based on its age
  const fadeOpacity = useMemo(() => {
    const age = Date.now() - timestamp;
    const fadeProgress = Math.min(age / FADE_DURATION_MS, 1);
    return Math.max(0, 1 - fadeProgress);
  }, [timestamp]);

  // If the point is completely faded, don't render it
  if (fadeOpacity <= 0) return null;

  return (
    <mesh position={position}>
      {/* White sphere representing the historical position */}
      <sphereGeometry args={[0.1, 8, 8]} />
      <meshBasicMaterial 
        color="white" 
        transparent 
        opacity={fadeOpacity}
      />
    </mesh>
  );
};

// Scanner Beam Component - This shows the LiDAR scanner beam as a line
// The beam changes color and style based on the system status
const ScannerBeam = ({ scannerPosition, status, measuredPosition }) => {
  // Calculate the beam color and style based on the current status
  const beamStyle = useMemo(() => {
    switch (status) {
      case 'TRACKING_TARGET':
        // When tracking, show a solid red beam pointing at the target
        return {
          color: '#ff4444',
          dashSize: 0,
          gapSize: 0,
          lineWidth: 3
        };
      case 'BACKGROUND_SCAN_IN_PROGRESS':
        // During background scan, show a cyan dashed beam following the scanner path
        return {
          color: '#00ffff',
          dashSize: 0.5,
          gapSize: 0.3,
          lineWidth: 2
        };
      case 'SEARCHING_FOR_TARGET':
        // When searching, show a yellow dashed beam following the search pattern
        return {
          color: '#ffff00',
          dashSize: 0.3,
          gapSize: 0.2,
          lineWidth: 2
        };
      default:
        // Default state - show a white beam
        return {
          color: '#ffffff',
          dashSize: 0,
          gapSize: 0,
          lineWidth: 1
        };
    }
  }, [status]);

  // If we don't have scanner position, don't render the beam
  if (!scannerPosition) return null;

  // Create points for the beam line (from origin to scanner position)
  const beamPoints = useMemo(() => {
    return [
      new THREE.Vector3(0, 0, 0),           // Start at LiDAR sensor (origin)
      new THREE.Vector3(...scannerPosition)  // End at scanner position
    ];
  }, [scannerPosition]);

  return (
    <Line
      points={beamPoints}
      color={beamStyle.color}
      lineWidth={beamStyle.lineWidth}
      dashed={beamStyle.dashSize > 0}
      dashSize={beamStyle.dashSize}
      gapSize={beamStyle.gapSize}
    />
  );
};

// LiDAR Sensor Component - This shows the LiDAR sensor as a red block at the center
// This represents the physical LiDAR device that's doing the scanning
const LidarSensor = () => {
  return (
    <mesh position={[0, 0, 0]}>
      {/* Red cube representing the LiDAR sensor */}
      <boxGeometry args={[0.5, 0.5, 0.5]} />
      <meshBasicMaterial color="#ff0000" />
    </mesh>
  );
};

// Range Sphere Component - This shows the maximum range of the LiDAR sensor
// It's a transparent sphere that helps users understand the sensor's coverage area
const RangeSphere = () => {
  return (
    <mesh position={[0, 0, 0]}>
      {/* Transparent sphere showing LiDAR range */}
      <sphereGeometry args={[LIDAR_RANGE_METERS, 32, 32]} />
      <meshBasicMaterial 
        color="#00ffff" 
        transparent 
        opacity={0.1}
        wireframe
      />
    </mesh>
  );
};

// Target Marker Component - This shows the current target position
// It appears as a green sphere when the system is tracking a target
const TargetMarker = ({ measuredPosition, status }) => {
  // Only show the target marker when we have a measured position
  if (!measuredPosition || status !== 'TRACKING_TARGET') return null;

  return (
    <mesh position={measuredPosition}>
      {/* Green sphere representing the current target position */}
      <sphereGeometry args={[0.3, 16, 16]} />
      <meshBasicMaterial color="#00ff00" />
    </mesh>
  );
};



// Live Telemetry Display Component - This shows real-time sensor data
// It displays the current pan/tilt angles, signal strength, and distance
const LiveTelemetry = ({ liveTelemetry }) => {
  // If we don't have telemetry data, don't render anything
  if (!liveTelemetry) return null;

  return (
    <Html
      position={[0, 8, 0]}
      center
      style={{
        background: 'rgba(0, 0, 0, 0.8)',
        color: 'white',
        padding: '15px',
        borderRadius: '8px',
        fontSize: '14px',
        fontFamily: 'monospace',
        border: '1px solid rgba(0, 255, 136, 0.5)',
        minWidth: '200px',
        textAlign: 'center'
      }}
    >
      <div style={{ marginBottom: '10px', fontWeight: 'bold', color: '#00ff88' }}>
        Live Telemetry
      </div>
      <div>Pan: {liveTelemetry.pan_angle?.toFixed(1) || 'N/A'}°</div>
      <div>Tilt: {liveTelemetry.tilt_angle?.toFixed(1) || 'N/A'}°</div>
      <div>Signal: {liveTelemetry.signal_strength?.toFixed(1) || 'N/A'}%</div>
      <div>Distance: {liveTelemetry.distance?.toFixed(2) || 'N/A'} m</div>
    </Html>
  );
};

// Legend Component - This explains what the different visual elements mean
// It helps users understand what they're looking at in the 3D scene
const Legend = () => {
  return (
    <Html
      position={[8, 0, 0]}
      center
      style={{
        background: 'rgba(0, 0, 0, 0.8)',
        color: 'white',
        padding: '15px',
        borderRadius: '8px',
        fontSize: '12px',
        border: '1px solid rgba(255, 255, 255, 0.3)',
        minWidth: '180px'
      }}
    >
      <div style={{ marginBottom: '10px', fontWeight: 'bold', color: '#00ff88' }}>
        Visual Legend
      </div>
      <div style={{ marginBottom: '5px' }}>
        <span style={{ color: 'red' }}>■</span> LiDAR Sensor
      </div>
      <div style={{ marginBottom: '5px' }}>
        <span style={{ color: 'lime' }}>●</span> Target Position
      </div>
      <div style={{ marginBottom: '5px' }}>
        <span style={{ color: 'white' }}>●</span> History (fades)
      </div>
      <div style={{ marginBottom: '5px' }}>
        <span style={{ color: 'cyan' }}>─</span> Scanner Beam
      </div>
      <div style={{ marginBottom: '5px' }}>
        <span style={{ color: 'red' }}>─</span> Tracking Beam
      </div>
      <div style={{ marginBottom: '5px' }}>
        <span style={{ color: 'cyan' }}>○</span> 12m Range Sphere
      </div>
      <div style={{ marginBottom: '5px' }}>
        <span style={{ color: '#ff6b35' }}>─</span> Predicted Orbit
      </div>
    </Html>
  );
};

// Main LocalView Component
const LocalView = ({
  measuredPosition,        // Current target position
  scannerPosition,         // Where the scanner beam is pointing
  positionHistory,         // History of target positions
  liveTelemetry,          // Real-time sensor data
  status,                 // Current system status
  progress,               // NEW: Progress percentage
  predictedOrbitParams    // Predicted orbit to show in local view
}) => {
  // Reference to the scanner beam for animations
  const scannerBeamRef = useRef();
  
  // Reference to the target mesh for animations
  const targetRef = useRef();
  
  // Reference to the LiDAR sensor mesh
  const lidarRef = useRef();

  // Animation frame for real-time updates
  useFrame(() => {
    // Animate the scanner beam if we have a scanner position
    if (scannerBeamRef.current && scannerPosition && scannerPosition[0] !== null) {
      // Update scanner beam position in real-time
      scannerBeamRef.current.position.set(...scannerPosition);
    }
    
    // Animate the target if we have a measured position
    if (targetRef.current && measuredPosition) {
      // Smoothly move the target to the new position
      targetRef.current.position.lerp(new THREE.Vector3(...measuredPosition), 0.1);
    }
    
    // Add subtle rotation to the LiDAR sensor
    if (lidarRef.current) {
      lidarRef.current.rotation.y += 0.01;
    }
  });

  // Calculate the range sphere radius based on the maximum range
  const rangeSphereRadius = useMemo(() => LIDAR_RANGE_METERS * SCENE_SCALE, []);

  return (
    <ErrorBoundary>
      {/* NEW: Real-Time Status Display */}
      <RealTimeStatus 
        status={status}
        progress={progress}
        message={liveTelemetry?.statusMessage}
        liveTelemetry={liveTelemetry}
      />
      
      {/* Grid for reference */}
      <Grid 
        args={[20, 20]} 
        cellSize={1} 
        cellThickness={0.5} 
        cellColor="#444444" 
        sectionSize={5} 
        sectionThickness={1} 
        sectionColor="#666666" 
        fadeDistance={25} 
        fadeStrength={1} 
        followCamera={false} 
        infiniteGrid={true} 
      />
      
      {/* LiDAR Sensor (Origin Point) */}
      <mesh ref={lidarRef} position={[0, 0, 0]}>
        <cylinderGeometry args={[0.3, 0.3, 0.5, 8]} />
        <meshBasicMaterial color="#00d4ff" />
      </mesh>
      
      {/* Scanner Beam */}
      <ScannerBeam 
        scannerPosition={scannerPosition}
        status={status}
        measuredPosition={measuredPosition}
      />
      
      {/* Range Sphere (Transparent) */}
      <mesh>
        <sphereGeometry args={[rangeSphereRadius, 16, 16]} />
        <meshBasicMaterial 
          color="#00d4ff" 
          transparent 
          opacity={0.1} 
          wireframe={true}
        />
      </mesh>
      
      {/* Current Target Position */}
      {measuredPosition && (
        <mesh ref={targetRef} position={measuredPosition}>
          <sphereGeometry args={[0.2, 12, 12]} />
          <meshBasicMaterial color="#ff6b35" />
        </mesh>
      )}
      
      {/* History Points */}
      {positionHistory && positionHistory.length > 0 && (
        <>
          {positionHistory.slice(-HISTORY_POINTS_MAX).map((historyPoint, index) => (
            <HistoryPoint
              key={`${historyPoint.time}-${index}`}
              position={historyPoint.pos}
              timestamp={historyPoint.time}
              index={index}
            />
          ))}
        </>
      )}
      
      {/* Predicted Orbit Path */}
      {predictedOrbitParams && (
        <PredictedOrbit 
          orbitParams={predictedOrbitParams}
          color="#00ff88"
          lineWidth={2}
        />
      )}
      
      {/* Coordinate System Axes */}
      <group>
        {/* X-axis (Red) */}
        <Line
          points={[[0, 0, 0], [5, 0, 0]]}
          color="#ff0000"
          lineWidth={2}
        />
        {/* Y-axis (Green) */}
        <Line
          points={[[0, 0, 0], [0, 5, 0]]}
          color="#00ff00"
          lineWidth={2}
        />
        {/* Z-axis (Blue) */}
        <Line
          points={[[0, 0, 0], [0, 0, 5]]}
          color="#0000ff"
          lineWidth={2}
        />
      </group>
    </ErrorBoundary>
  );
};

// Export the LocalView component so it can be used by the main App component
export default LocalView;
