import React, { useMemo } from 'react';
import { Line } from '@react-three/drei';
import * as THREE from 'three';

// OrbitPath Component - This renders the predicted orbit path for satellites
// It shows where satellites will travel around the Earth in the future
const OrbitPath = ({ orbitParams, color = '#00ff88', segments = 100 }) => {
  // Generate points along the orbit path
  const orbitPoints = useMemo(() => {
    const points = [];
    
    // If we don't have orbit parameters, create a simple circular orbit
    if (!orbitParams || !orbitParams.elements) {
      // Default circular orbit around Earth
      const radius = 7; // 7,000 km in scene units
      for (let i = 0; i <= segments; i++) {
        const angle = (i / segments) * Math.PI * 2;
        const x = radius * Math.cos(angle);
        const z = radius * Math.sin(angle);
        points.push(new THREE.Vector3(x, 0, z));
      }
    } else {
      // Use provided orbital elements to generate realistic orbit
      const { elements } = orbitParams;
      const { semiMajorAxis, eccentricity, inclinationRad } = elements;
      
      // Convert orbital parameters to scene coordinates
      const radius = (semiMajorAxis || 7000) / 1000; // Convert km to scene units
      const ecc = eccentricity || 0.001;
      const inclination = inclinationRad || 0;
      
      for (let i = 0; i <= segments; i++) {
        const angle = (i / segments) * Math.PI * 2;
        
        // Calculate position along elliptical orbit
        const r = radius * (1 - ecc * ecc) / (1 + ecc * Math.cos(angle));
        const x = r * Math.cos(angle);
        const z = r * Math.sin(angle);
        
        // Apply inclination (tilt of the orbit)
        const y = z * Math.sin(inclination);
        const zFinal = z * Math.cos(inclination);
        
        points.push(new THREE.Vector3(x, y, zFinal));
      }
    }
    
    return points;
  }, [orbitParams, segments]);

  // Don't render if we don't have any points
  if (orbitPoints.length === 0) return null;

  return (
    <Line
      points={orbitPoints}
      color={color}
      lineWidth={2}
      dashed={true}
      dashSize={0.5}
      gapSize={0.3}
      transparent
      opacity={0.7}
    />
  );
};

export default OrbitPath;
