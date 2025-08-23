// src/components/Scene.jsx (Reverted to simple version)
import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Line } from '@react-three/drei';
import * as THREE from 'three';
import useDroneStore from '../useDroneStore';

const FADE_OUT_TIME = 10000;

function HistoryPoint({ data }) {
  const meshRef = useRef();
  useFrame(() => {
    if (!meshRef.current) return;
    const age = Date.now() - data.time;
    const opacity = Math.max(0, 1 - age / FADE_OUT_TIME);
    meshRef.current.material.opacity = opacity;
    meshRef.current.visible = opacity > 0.01;
  });
  return (
    <mesh ref={meshRef} position={data.pos}>
      <sphereGeometry args={[0.05, 8, 8]} />
      <meshStandardMaterial color="white" transparent />
    </mesh>
  );
}

function PredictedOrbit() {
    const params = useDroneStore((state) => state.payload.predicted_orbit_params);
    if (!params || params.type !== 'ellipse') return null;
    const curve = new THREE.EllipseCurve(0, 0, params.a, params.b, 0, 2 * Math.PI, false, 0);
    const points = curve.getPoints(100).map(p => new THREE.Vector3(p.x, params.altitude, p.y));
    return <Line points={points} color="blue" lineWidth={2} />;
}

export default function Scene() {
  const { measured_pos, position_history } = useDroneStore((state) => state.payload);
  
  return (
    <>
      <ambientLight intensity={0.5} />
      <directionalLight position={[10, 10, 5]} intensity={1} />
      <OrbitControls />
      <axesHelper args={[2]} />
      <Grid infiniteGrid cellSize={1} sectionSize={5} fadeDistance={50} fadeStrength={1.5} />

      <mesh position={[0, 0, 0]}>
        <boxGeometry args={[0.2, 0.2, 0.2]} />
        <meshStandardMaterial color="red" />
      </mesh>
      <mesh>
        <sphereGeometry args={[12, 32, 32]} />
        <meshStandardMaterial color="cyan" wireframe transparent opacity={0.1} />
      </mesh>
      
      {measured_pos && measured_pos[0] !== null && (
        <mesh position={measured_pos}>
          <sphereGeometry args={[0.15, 16, 16]} />
          <meshStandardMaterial color="lime" emissive="green" emissiveIntensity={2} />
        </mesh>
      )}

      {position_history.map((data, index) => (
        <HistoryPoint key={index} data={data} />
      ))}
      
      <PredictedOrbit />
    </>
  );
}