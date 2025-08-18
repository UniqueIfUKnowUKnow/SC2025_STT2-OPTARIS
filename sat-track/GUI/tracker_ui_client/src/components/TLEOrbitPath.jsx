import React, { useMemo } from 'react';
import { Line } from '@react-three/drei';
import * as THREE from 'three';
import * as satellite from 'satellite.js';

// Render an orbit path by propagating a TLE satrec over time
const TLEOrbitPath = ({ satrec, color = '#3399ff', segments = 180, durationMinutes = 90, startTime }) => {
  const points = useMemo(() => {
    if (!satrec) return [];
    const pts = [];
    const start = startTime instanceof Date ? startTime : new Date();
    for (let i = 0; i <= segments; i++) {
      const dtMin = (i / segments) * durationMinutes;
      const t = new Date(start.getTime() + dtMin * 60 * 1000);
      try {
        const pv = satellite.propagate(satrec, t);
        if (pv && pv.position) {
          const { x, y, z } = pv.position; // km in TEME
          pts.push(new THREE.Vector3(x / 1000, y / 1000, z / 1000));
        }
      } catch (_) {
        // skip bad samples
      }
    }
    return pts;
  }, [satrec, segments, durationMinutes, startTime]);

  if (!points.length) return null;

  return (
    <Line
      points={points}
      color={color}
      lineWidth={2}
      dashed={false}
      transparent
      opacity={0.9}
    />
  );
};

export default TLEOrbitPath;


