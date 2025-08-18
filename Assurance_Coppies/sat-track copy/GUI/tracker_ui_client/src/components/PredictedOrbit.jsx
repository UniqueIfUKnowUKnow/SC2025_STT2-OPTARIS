import React, { useMemo } from 'react';
import { Line } from '@react-three/drei';
import * as THREE from 'three';

// PredictedOrbit Component - This renders the predicted orbit path based on LiDAR measurements
// It shows the calculated orbit that the target object will follow
const PredictedOrbit = ({ predictedOrbitParams, status }) => {
  // Only show the predicted orbit when we're actively tracking a target
  const shouldShow = status === 'TRACKING_TARGET' && predictedOrbitParams;
  
  // Generate points along the predicted orbit path
  const orbitPoints = useMemo(() => {
    if (!shouldShow || !predictedOrbitParams) return [];
    
    const points = [];
    const segments = 120; // Number of points to generate for smooth orbit
    
    // Extract orbital parameters from the LiDAR measurements
    const {
      semi_major_axis,
      eccentricity,
      inclination,
      raan, // Right Ascension of Ascending Node
      argument_of_periapsis,
      true_anomaly
    } = predictedOrbitParams;
    
    // Convert to scene coordinates (scale down for visibility)
    const radius = (semi_major_axis || 7000) / 1000; // Convert km to scene units
    const ecc = eccentricity || 0.001;
    const inc = (inclination || 0) * Math.PI / 180; // Convert degrees to radians
    const raanRad = (raan || 0) * Math.PI / 180;
    const argPerRad = (argument_of_periapsis || 0) * Math.PI / 180;
    
    // Generate points along the elliptical orbit
    for (let i = 0; i <= segments; i++) {
      const angle = (i / segments) * Math.PI * 2;
      
      // Calculate position along elliptical orbit
      const r = radius * (1 - ecc * ecc) / (1 + ecc * Math.cos(angle));
      
      // Basic elliptical coordinates
      let x = r * Math.cos(angle);
      let y = r * Math.sin(angle);
      let z = 0;
      
      // Apply orbital inclination (tilt)
      const yRotated = y * Math.cos(inc) - z * Math.sin(inc);
      const zRotated = y * Math.sin(inc) + z * Math.cos(inc);
      y = yRotated;
      z = zRotated;
      
      // Apply RAAN rotation (rotation around Z-axis)
      const xRa = x * Math.cos(raanRad) - y * Math.sin(raanRad);
      const yRa = x * Math.sin(raanRad) + y * Math.cos(raanRad);
      x = xRa;
      y = yRa;
      
      // Apply argument of periapsis rotation
      const xArg = x * Math.cos(argPerRad) - y * Math.sin(argPerRad);
      const yArg = x * Math.sin(argPerRad) + y * Math.cos(argPerRad);
      x = xArg;
      y = yArg;
      
      points.push(new THREE.Vector3(x, y, z));
    }
    
    return points;
  }, [predictedOrbitParams, status, shouldShow]);

  // Don't render if we don't have any points or shouldn't show
  if (!shouldShow || orbitPoints.length === 0) return null;

  return (
    <Line
      points={orbitPoints}
      color="#00ff88" // Green color for predicted orbit
      lineWidth={3}
      dashed={false} // Solid line for predicted orbit
      transparent
      opacity={0.8}
    />
  );
};

export default PredictedOrbit;
