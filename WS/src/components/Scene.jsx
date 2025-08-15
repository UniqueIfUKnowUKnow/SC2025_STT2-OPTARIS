// src/components/Scene.jsx
import React, { useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Line } from '@react-three/drei';
import * as THREE from 'three';
import { useStore } from '../useDroneStore';
import { useWebSocket } from '../hooks/useWebSocket';

const FADE_OUT_TIME = 10000;

function Scene() {
  const coordinates = useStore((state) => state.coordinates);
  const prediction = useStore((state) => state.prediction);
  const { isConnected, sendMessage } = useWebSocket();

  // Reference to store history points
  const historyPoints = useRef([]);

  useEffect(() => {
    // Request initial data when connected
    if (isConnected) {
      sendMessage({ command: 'get_coordinates' });
      sendMessage({ command: 'get_prediction' });
    }
  }, [isConnected, sendMessage]);

  // Update scene on every frame
  useFrame((state) => {
    if (coordinates) {
      // Add new point to history
      historyPoints.current.push({
        position: new THREE.Vector3(
          coordinates.x,
          coordinates.y,
          coordinates.z
        ),
        timestamp: Date.now()
      });

      // Remove old points
      const currentTime = Date.now();
      historyPoints.current = historyPoints.current.filter(
        point => currentTime - point.timestamp < FADE_OUT_TIME
      );
    }
  });

  return (
    <>
      <OrbitControls />
      <Grid 
        infiniteGrid
        fadeDistance={50}
        fadeStrength={1}
      />
      
      {/* Current Position */}
      {coordinates && (
        <mesh position={[coordinates.x, coordinates.y, coordinates.z]}>
          <sphereGeometry args={[0.5, 32, 32]} />
          <meshStandardMaterial color="red" />
        </mesh>
      )}

      {/* History Trail */}
      {historyPoints.current.map((point, index) => (
        <mesh key={index} position={point.position.toArray()}>
          <sphereGeometry args={[0.1, 8, 8]} />
          <meshStandardMaterial 
            color="blue" 
            opacity={1 - (Date.now() - point.timestamp) / FADE_OUT_TIME}
            transparent
          />
        </mesh>
      ))}

      {/* Predicted Trajectory */}
      {prediction && (
        <Line
          points={prediction.points}
          color="green"
          lineWidth={2}
        />
      )}

      {/* Lights */}
      <ambientLight intensity={0.5} />
      <directionalLight position={[10, 10, 5]} intensity={1} />
    </>
  );
}

export default Scene;


      </mesh>
      
      {/* Conditionally render the current drone position if it exists and is valid */}
      {measured_pos && measured_pos[0] !== null && (
        <mesh position={measured_pos}>
          {/* Create a sphere geometry for the drone with radius 0.15 units */}
          <sphereGeometry args={[0.15, 16, 16]} />
          {/* Lime green material with green emissive glow to make it stand out */}
          <meshStandardMaterial color="lime" emissive="green" emissiveIntensity={2} />
        </mesh>
      )}

      {/* Render all history points by mapping over the position history array */}
      {/* Each history point shows where the drone was at a previous time */}
      {position_history.map((data, index) => (
        <HistoryPoint key={index} data={data} />
      ))}
      
      {/* Render the initial orbit (from TLE) in orange if available */}
      <OrbitLine params={initial_orbit_params} color="orange" />
      {/* Remove predicted path line */}
      {/* <OrbitLine params={predicted_orbit_params} color="blue" /> */}

      {/* Render the scanner beam line to current scanner position */}
      <ScannerBeam />
    </>
  );
}