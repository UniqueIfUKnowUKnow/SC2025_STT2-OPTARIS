import React, { useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

const Starfield = ({ count = 5000, radius = 300 }) => {
  const starsRef = useRef();
  
  // Generate random star positions
  const { positions, colors } = useMemo(() => {
    const positions = new Float32Array(count * 3);
    const colors = new Float32Array(count * 3);
    
    for (let i = 0; i < count; i++) {
      // Generate random spherical coordinates
      const theta = Math.random() * Math.PI * 2;
      const phi = Math.acos(Math.random() * 2 - 1);
      
      // Convert to Cartesian coordinates
      const x = radius * Math.sin(phi) * Math.cos(theta);
      const y = radius * Math.sin(phi) * Math.sin(theta);
      const z = radius * Math.cos(phi);
      
      positions[i * 3] = x;
      positions[i * 3 + 1] = y;
      positions[i * 3 + 2] = z;
      
      // Random star colors (white to blue-ish)
      const intensity = 0.5 + Math.random() * 0.5;
      colors[i * 3] = intensity; // Red
      colors[i * 3 + 1] = intensity; // Green
      colors[i * 3 + 2] = intensity + Math.random() * 0.3; // Blue (slightly more blue)
    }
    
    return { positions, colors };
  }, [count, radius]);
  
  // Gentle rotation animation
  useFrame(({ clock }) => {
    if (starsRef.current) {
      starsRef.current.rotation.y = clock.getElapsedTime() * 0.0005;
      starsRef.current.rotation.x = clock.getElapsedTime() * 0.0003;
    }
  });
  
  return (
    <points ref={starsRef}>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          count={count}
          array={positions}
          itemSize={3}
        />
        <bufferAttribute
          attach="attributes-color"
          count={count}
          array={colors}
          itemSize={3}
        />
      </bufferGeometry>
      <pointsMaterial
        size={1.5}
        sizeAttenuation={false}
        vertexColors
        transparent
        opacity={0.8}
        blending={THREE.AdditiveBlending}
      />
    </points>
  );
};

export default Starfield;
