import React, { useRef } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import { TextureLoader } from 'three';
import * as THREE from 'three';

const SimpleEarth = ({ radius = 6.371 }) => {
  const earthRef = useRef();
  
  // Load both day texture and normal map for maximum realism
  const [dayTexture, normalTexture] = useLoader(TextureLoader, [
    '/textures/earth_daymap.jpg',
    '/textures/earth_normalmap.jpg'
  ]);

  // Real Earth rotation speed calculation
  // Earth rotates 360° in 24 hours = 15°/hour = 0.25°/minute = 0.00417°/second
  // At 60 FPS: 0.00417° / 60 = 0.0000695° per frame
  // Convert to radians: 0.0000695° × (π/180) = 0.00000121 radians per frame
  const earthRotationSpeed = (2 * Math.PI) / (24 * 60 * 60 * 60); // radians per frame at 60fps
  
  useFrame(() => {
    if (earthRef.current) {
      earthRef.current.rotation.y += earthRotationSpeed;
    }
  });

  return (
    <mesh ref={earthRef}>
      <sphereGeometry args={[radius, 128, 64]} />
      <meshStandardMaterial 
        map={dayTexture} // Realistic Earth surface texture
        normalMap={normalTexture} // Surface elevation detail
        normalScale={[1.5, 1.5]} // Natural terrain detail
        roughness={0.7} // Natural surface roughness
        metalness={0.0} // No metalness for Earth
        // Very subtle enhancement
        emissiveIntensity={0.0} // No artificial glow for maximum realism
      />
    </mesh>
  );
};

export default SimpleEarth;
