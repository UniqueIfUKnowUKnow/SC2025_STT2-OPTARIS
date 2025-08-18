import React, { useMemo } from 'react';
import { Line } from '@react-three/drei';
import * as THREE from 'three';

// Render a polyline of sampled positions [{pos:[x,y,z], time}]
const TrajectoryLine = ({ samples, color = '#ffcc00', width = 2, dashed = true }) => {
  const points = useMemo(() => {
    if (!Array.isArray(samples) || samples.length === 0) return [];
    return samples.map(s => new THREE.Vector3(...s.pos));
  }, [samples]);

  if (!points.length) return null;

  return (
    <Line
      points={points}
      color={color}
      lineWidth={width}
      dashed={dashed}
      dashSize={0.4}
      gapSize={0.25}
      transparent
      opacity={0.85}
    />
  );
};

export default TrajectoryLine;


