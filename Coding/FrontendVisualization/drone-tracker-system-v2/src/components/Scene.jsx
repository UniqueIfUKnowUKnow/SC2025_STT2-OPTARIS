// src/components/Scene.jsx (Reverted to simple version)
// This file contains the main 3D scene component for the drone tracker visualization
// It renders a 3D environment with the drone's current position, flight history, predicted orbit, and scanner beam

// Import React hooks and core functionality
import React, { useRef, useMemo } from 'react';
// Import useFrame hook for animation loop integration with React Three Fiber
import { useFrame } from '@react-three/fiber';
// Import 3D components and utilities from React Three Drei library
import { OrbitControls, Grid, Line } from '@react-three/drei';
// Import Three.js library for 3D graphics and math operations
import * as THREE from 'three';
// Import our custom store for managing drone state and data
import useDroneStore from '../useDroneStore';

// Define constant for how long history points should fade out (10 seconds in milliseconds)
const FADE_OUT_TIME = 10000;

// Component to render individual history points showing where the drone was in the past
// Each point fades out over time to show the drone's recent flight path
function HistoryPoint({ data }) {
  // Create a reference to the mesh object for direct manipulation
  const meshRef = useRef();
  
  // This hook runs on every frame (60fps) to update the visual properties
  useFrame(() => {
    // Safety check: if the mesh reference doesn't exist, exit early
    if (!meshRef.current) return;
    
    // Calculate how old this history point is (current time minus when it was recorded)
    const age = Date.now() - data.time;
    
    // Calculate opacity based on age - starts at 1 (fully visible) and fades to 0
    // Math.max ensures opacity never goes below 0
    const opacity = Math.max(0, 1 - age / FADE_OUT_TIME);
    
    // Apply the calculated opacity to the material
    meshRef.current.material.opacity = opacity;
    
    // Hide the mesh completely when opacity gets very low (below 1%) for performance
    meshRef.current.visible = opacity > 0.01;
  });
  
  // Return the 3D mesh representing this history point
  return (
    <mesh ref={meshRef} position={data.pos}>
      {/* Create a small sphere geometry with radius 0.05 and 8 segments for smooth appearance */}
      <sphereGeometry args={[0.05, 8, 8]} />
      {/* White material that supports transparency for the fade effect */}
      <meshStandardMaterial color="white" transparent />
    </mesh>
  );
}

// Renders an orbit line given ellipse parameters { a, b, altitude }
function OrbitLine({ params, color = 'blue' }) {
  // Do not render if params are missing or not an ellipse
  if (!params || params.type !== 'ellipse') return null;
  // Build an ellipse at the given altitude
  const curve = new THREE.EllipseCurve(0, 0, params.a, params.b, 0, 2 * Math.PI, false, 0);
  // Convert 2D ellipse to 3D points by placing it on a constant Y plane
  const points = curve.getPoints(100).map((p) => new THREE.Vector3(p.x, params.altitude, p.y));
  // Render a line with the requested color
  return <Line points={points} color={color} lineWidth={2} />;
}

// Component to render the scanner beam line from origin to scanner_pos
function ScannerBeam() {
  // Select status and scanner_pos from the store
  const status = useDroneStore((state) => state.status);
  const scanner_pos = useDroneStore((state) => state.payload.scanner_pos);

  // Validate scanner_pos: must be an array of 3 finite numbers
  const isValid = Array.isArray(scanner_pos) && scanner_pos.length === 3 && scanner_pos.every((v) => typeof v === 'number' && Number.isFinite(v));
  if (!isValid) return null;

  // Prepare points for the line: from origin to scanner_pos
  const points = useMemo(() => [new THREE.Vector3(0, 0, 0), new THREE.Vector3(scanner_pos[0], scanner_pos[1], scanner_pos[2])], [scanner_pos]);

  // Determine styling based on status
  const isTracking = status === 'TRACKING_TARGET';
  const color = isTracking ? 'red' : 'cyan';
  const lineWidth = isTracking ? 3 : 1.5;
  const dashed = !isTracking;

  return (
    <Line points={points} color={color} lineWidth={lineWidth} dashed={dashed} dashSize={0.5} gapSize={0.25} />
  );
}

// Main Scene component that renders the entire 3D environment
export default function Scene() {
  // Extract current drone position and position history from the store
  const { measured_pos, position_history, initial_orbit_params } = useDroneStore((state) => state.payload);
  
  return (
    <>
      {/* Add ambient lighting to provide overall illumination to the scene */}
      <ambientLight intensity={0.5} />
      
      {/* Add directional lighting from above and to the side for shadows and depth */}
      <directionalLight position={[10, 10, 5]} intensity={1} />
      
      {/* Add camera controls allowing user to orbit, zoom, and pan around the scene */}
      <OrbitControls />
      
      {/* Add coordinate axes helper (red=X, green=Y, blue=Z) for reference */}
      <axesHelper args={[2]} />
      
      {/* Add an infinite grid on the ground plane for spatial reference */}
      {/* cellSize=1 means 1 unit squares, sectionSize=5 means major grid lines every 5 units */}
      <Grid infiniteGrid cellSize={1} sectionSize={5} fadeDistance={50} fadeStrength={1.5} />

      {/* Render a small red cube at the origin (0,0,0) as a reference point */}
      <mesh position={[0, 0, 0]}>
        {/* Create a cube geometry with sides of length 0.2 units */}
        <boxGeometry args={[0.2, 0.2, 0.2]} />
        {/* Red material for the cube */}
        <meshStandardMaterial color="red" />
      </mesh>
      
      {/* Render a large wireframe sphere representing the Earth or boundary sphere */}
      <mesh>
        {/* Create a sphere with radius 12 units and 32 segments for smooth appearance */}
        <sphereGeometry args={[12, 32, 32]} />
        {/* Cyan wireframe material with low opacity for subtle appearance */}
        <meshStandardMaterial color="cyan" wireframe transparent opacity={0.1} />
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