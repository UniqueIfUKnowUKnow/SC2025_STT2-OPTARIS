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
    </Html>
  );
};

// Main LocalView Component - This renders the entire 3D LiDAR tracker scene
const LocalView = ({
  measuredPosition,              // Current target position from LiDAR measurements
  scannerPosition,               // Where the scanner beam is currently pointing
  positionHistory,               // History of previous target positions
  liveTelemetry,                // Real-time sensor data (angles, signal strength, etc.)
  status,                        // Current system status
  predictedOrbitParams           // Calculated orbital parameters for the target
}) => {
  // Reference to the scene group for managing the 3D objects
  const sceneRef = useRef();

  // Filter and sort position history to show only recent points
  const recentHistory = useMemo(() => {
    if (!positionHistory || positionHistory.length === 0) return [];
    
    // Sort by timestamp (newest first) and take only the most recent points
    const sorted = [...positionHistory]
      .sort((a, b) => b.timestamp - a.timestamp)
      .slice(0, HISTORY_POINTS_MAX);
    
    return sorted;
  }, [positionHistory]);

  return (
    <group ref={sceneRef}>
      {/* Lighting for the 3D scene */}
      <ambientLight intensity={0.6} />
      <directionalLight position={[10, 10, 5]} intensity={0.8} />
      
      {/* Ground grid to help with spatial orientation */}
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
      
      {/* LiDAR sensor at the center */}
      <LidarSensor />
      
      {/* Range sphere showing maximum LiDAR coverage */}
      <RangeSphere />
      
      {/* Scanner beam showing where the LiDAR is pointing */}
      <ScannerBeam 
        scannerPosition={scannerPosition}
        status={status}
        measuredPosition={measuredPosition}
      />
      
      {/* Target marker showing current measured position */}
      <TargetMarker 
        measuredPosition={measuredPosition}
        status={status}
      />
      
      {/* History points showing previous target positions */}
      {recentHistory.map((point, index) => (
        <HistoryPoint
          key={`history-${index}`}
          position={point.position}
          timestamp={point.timestamp}
          index={index}
        />
      ))}
      
      {/* Predicted orbit based on LiDAR measurements */}
      <PredictedOrbit 
        predictedOrbitParams={predictedOrbitParams}
        status={status}
      />
      
      {/* Live telemetry display */}
      {/* Visual legend */}
         {/* Live telemetry display and visual legend overlays removed as requested */}
    </group>
  );
};

// Export the LocalView component so it can be used by the main App component
export default LocalView;
