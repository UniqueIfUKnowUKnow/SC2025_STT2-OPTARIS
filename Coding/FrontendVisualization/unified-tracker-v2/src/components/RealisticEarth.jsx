import React, { useRef, useMemo } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import { TextureLoader } from 'three';
import * as THREE from 'three';

const RealisticEarth = ({ radius = 6.371 }) => {
  const earthRef = useRef();
  
  // Load the Earth day texture
  const dayTexture = useLoader(TextureLoader, '/textures/earth_daymap.jpg');

  // Create material with the Earth texture
  const earthMaterial = useMemo(() => {
    if (dayTexture) {
      // Configure texture
      dayTexture.wrapS = dayTexture.wrapT = THREE.RepeatWrapping;
      
      return new THREE.MeshStandardMaterial({ 
        map: dayTexture,
        roughness: 0.8,
        metalness: 0.1,
      });
    }
    
    // Fallback material if texture fails to load
    return new THREE.MeshStandardMaterial({ 
      color: '#2E7D32', // Earth green
      roughness: 0.8,
      metalness: 0.1,
    });
  }, [dayTexture]);

  // Animation for Earth rotation
  useFrame(({ clock }) => {
    if (earthRef.current) {
      // Slow Earth rotation (one rotation per day)
      earthRef.current.rotation.y = clock.getElapsedTime() * 0.005;
    }
  });

  // Create geometry
  const earthGeometry = useMemo(() => {
    return new THREE.SphereGeometry(radius, 128, 64);
  }, [radius]);

  return (
    <group>
      {/* Main Earth sphere with texture */}
      <mesh ref={earthRef} material={earthMaterial} geometry={earthGeometry} />
      
      {/* Atmospheric glow */}
      <mesh>
        <sphereGeometry args={[radius * 1.008, 64, 32]} />
        <meshBasicMaterial 
          color="#4488ff"
          transparent
          opacity={0.08}
          side={THREE.BackSide}
          blending={THREE.AdditiveBlending}
        />
      </mesh>
      
      {/* Outer atmospheric ring */}
      <mesh>
        <sphereGeometry args={[radius * 1.015, 32, 16]} />
        <meshBasicMaterial 
          color="#66aaff"
          transparent
          opacity={0.02}
          side={THREE.BackSide}
          blending={THREE.AdditiveBlending}
        />
      </mesh>
    </group>
  );
};

export default RealisticEarth;
