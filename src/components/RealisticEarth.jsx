import React, { useRef, useMemo } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import { TextureLoader } from 'three';
import * as THREE from 'three';

// Enhanced Realistic Earth Shader with Normal Mapping and Atmospheric Effects
const RealisticEarthShader = {
  vertexShader: `
    varying vec2 vUv;
    varying vec3 vNormal;
    varying vec3 vPosition;
    varying vec3 vWorldPosition;
    varying vec3 vViewPosition;
    varying vec3 vTangent;
    varying vec3 vBitangent;
    
    attribute vec3 tangent;
    
    void main() {
      vUv = uv;
      vNormal = normalize(normalMatrix * normal);
      vPosition = position;
      vWorldPosition = (modelMatrix * vec4(position, 1.0)).xyz;
      vViewPosition = (modelViewMatrix * vec4(position, 1.0)).xyz;
      
      // Calculate tangent space vectors for normal mapping
      vTangent = normalize(normalMatrix * tangent);
      vBitangent = cross(vNormal, vTangent);
      
      gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    }
  `,
  
  fragmentShader: `
    uniform sampler2D dayTexture;
    uniform sampler2D normalTexture;
    uniform vec3 sunDirection;
    uniform float time;
    uniform vec3 cameraPosition;
    
    varying vec2 vUv;
    varying vec3 vNormal;
    varying vec3 vPosition;
    varying vec3 vWorldPosition;
    varying vec3 vViewPosition;
    varying vec3 vTangent;
    varying vec3 vBitangent;
    
    // Fresnel function for atmospheric rim lighting
    float fresnel(vec3 viewDirection, vec3 normal, float power) {
      return pow(1.0 - max(0.0, dot(viewDirection, normal)), power);
    }
    
    void main() {
      // Sample textures
      vec3 dayColor = texture2D(dayTexture, vUv).rgb;
      vec3 normalMap = texture2D(normalTexture, vUv).rgb;
      
      // Convert normal map from [0,1] to [-1,1] range
      normalMap = normalMap * 2.0 - 1.0;
      
      // Create TBN matrix for normal mapping
      mat3 tbn = mat3(
        normalize(vTangent),
        normalize(vBitangent),
        normalize(vNormal)
      );
      
      // Apply normal mapping
      vec3 normal = normalize(tbn * normalMap);
      
      // Calculate sun angle with enhanced normal
      float sunAngle = dot(normal, normalize(sunDirection));
      
      // Enhanced lighting calculation
      float lightIntensity = max(0.2, sunAngle); // Minimum ambient lighting
      
      // Ocean specular highlights - enhanced for better normal map visibility
      float oceanMask = smoothstep(0.0, 0.1, length(normalMap.xy)); // Detect water areas
      float specular = pow(max(0.0, dot(reflect(-normalize(sunDirection), normal), normalize(cameraPosition - vWorldPosition))), 64.0);
      vec3 specularColor = vec3(0.8, 0.9, 1.0) * specular * oceanMask * 2.0;
      
      // Enhanced base color with normal map influence
      vec3 color = dayColor * lightIntensity;
      
      // Add specular highlights
      color += specularColor;
      
      // Atmospheric rim lighting
      vec3 viewDirection = normalize(cameraPosition - vWorldPosition);
      float rim = fresnel(viewDirection, vNormal, 2.0);
      vec3 atmosphereColor = vec3(0.3, 0.6, 1.0) * rim * 0.3;
      
      // Enhance normal map contribution for better surface detail visibility
      color += normalMap.rgb * 0.15; // Increased from 0.05 for better normal map visibility
      
      // Add subtle terrain elevation effects based on normal map
      float elevation = (normalMap.r + normalMap.g + normalMap.b) / 3.0;
      color += vec3(0.1, 0.08, 0.05) * elevation * 0.5; // Brown tint for mountains
      
      // Final color with atmosphere
      color += atmosphereColor;
      
      gl_FragColor = vec4(color, 1.0);
    }
  `
};

const RealisticEarth = ({ radius = 6.371 }) => {
  const earthRef = useRef();
  
  // Use React Three Fiber's useLoader hook - this is the proven approach
  const [dayTexture, normalTexture] = useLoader(TextureLoader, [
    '/textures/earth_daymap.jpg',
    '/textures/earth_normalmap.jpg'
  ]);

  // Create enhanced shader material
  const earthMaterial = useMemo(() => {
    console.log('Creating earth material with loaded textures');
    
    if (dayTexture && normalTexture) {
      // Configure textures
      dayTexture.wrapS = dayTexture.wrapT = THREE.RepeatWrapping;
      normalTexture.wrapS = normalTexture.wrapT = THREE.RepeatWrapping;
      
      return new THREE.ShaderMaterial({
        vertexShader: RealisticEarthShader.vertexShader,
        fragmentShader: RealisticEarthShader.fragmentShader,
        uniforms: {
          dayTexture: { value: dayTexture },
          normalTexture: { value: normalTexture },
          sunDirection: { value: new THREE.Vector3(1, 0, 0) },
          time: { value: 0 },
          cameraPosition: { value: new THREE.Vector3(0, 0, 0) }
        }
      });
    }
    
    // Fallback material
    return new THREE.MeshStandardMaterial({ 
      color: '#2E7D32', // Earth green
      roughness: 0.8,
      metalness: 0.1,
    });
  }, [dayTexture, normalTexture]);

  // Animation and shader uniform updates
  useFrame(({ clock, camera }) => {
    if (earthRef.current) {
      // Slow Earth rotation
      earthRef.current.rotation.y = clock.getElapsedTime() * 0.005;
    }

    // Update shader uniforms
    if (earthMaterial.uniforms) {
      // Update sun direction for day/night cycle
      if (earthMaterial.uniforms.sunDirection) {
        const time = clock.getElapsedTime() * 0.05;
        earthMaterial.uniforms.sunDirection.value.set(
          Math.cos(time),
          0.2,
          Math.sin(time)
        );
      }

      // Update time uniform
      if (earthMaterial.uniforms.time) {
        earthMaterial.uniforms.time.value = clock.getElapsedTime();
      }

      // Update camera position for specular highlights
      if (earthMaterial.uniforms.cameraPosition) {
        earthMaterial.uniforms.cameraPosition.value.copy(camera.position);
      }
    }
  });

  // Create geometry with tangent attributes for normal mapping
  const earthGeometry = useMemo(() => {
    const geometry = new THREE.SphereGeometry(radius, 256, 128);
    geometry.computeTangents();
    return geometry;
  }, [radius]);

  return (
    <group>
      {/* Main Earth sphere with enhanced geometry */}
      <mesh ref={earthRef} material={earthMaterial} geometry={earthGeometry} />
      
      {/* Enhanced atmospheric glow */}
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
